/*
 * SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth_driver.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "ch390.h"
#include "esp_eth_mac_ch390.h"

static const char *TAG = "ch390.mac";

#define CH390_SPI_LOCK_TIMEOUT_MS (50)
#define CH390_PHY_OPERATION_TIMEOUT_US (1000)
#define CH390_RX_MEM_START_ADDR (3072)
#define CH390_RX_MEM_MAX_SIZE (16384)
#define CH390_RX_HDR_SIZE (4)
#define CH390_ETH_MAC_RX_BUF_SIZE_AUTO (0)

typedef struct {
    uint32_t copy_len;
    uint32_t byte_cnt;
} __attribute__((packed)) ch390_auto_buf_info_t;

typedef struct {
    uint8_t flag;
    uint8_t status;
    uint8_t length_low;
    uint8_t length_high;
} ch390_rx_header_t;

typedef struct {
    spi_device_handle_t hdl;
    SemaphoreHandle_t lock;
} eth_spi_info_t;

typedef struct {
    void *ctx;
    void *(*init)(const void *spi_config);
    esp_err_t (*deinit)(void *spi_ctx);
    esp_err_t (*read)(void *spi_ctx, uint32_t cmd, uint32_t addr, void *data, uint32_t data_len);
    esp_err_t (*write)(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *data, uint32_t data_len);
} eth_spi_custom_driver_t;

typedef struct {
    esp_eth_mac_t parent;
    esp_eth_mediator_t *eth;
    eth_spi_custom_driver_t spi;
    TaskHandle_t rx_task_hdl;
    uint32_t sw_reset_timeout_ms;
    int int_gpio_num;
    esp_timer_handle_t poll_timer;
    uint32_t poll_period_ms;
    uint8_t addr[6];
    bool packets_remain;
    bool flow_ctrl_enabled;
    uint8_t *rx_buffer;
} emac_ch390_t;

static void *ch390_spi_init(const void *spi_config)
{
    void *ret = NULL;
    eth_ch390_config_t *ch390_config = (eth_ch390_config_t *)spi_config;
    eth_spi_info_t *spi = calloc(1, sizeof(eth_spi_info_t));
    ESP_GOTO_ON_FALSE(spi, NULL, err, TAG, "no memory for SPI context data");

    /* SPI device init */
    spi_device_interface_config_t spi_devcfg;
    spi_devcfg = *(ch390_config->spi_devcfg);
    if (ch390_config->spi_devcfg->command_bits == 0 && ch390_config->spi_devcfg->address_bits == 0) {
        /* configure default SPI frame format */
        spi_devcfg.command_bits = 1;
        spi_devcfg.address_bits = 7;
    } else {
        ESP_GOTO_ON_FALSE(ch390_config->spi_devcfg->command_bits == 1 && ch390_config->spi_devcfg->address_bits == 7,
                          NULL, err, TAG, "incorrect SPI frame format (command_bits/address_bits)");
    }
    ESP_GOTO_ON_FALSE(spi_bus_add_device(ch390_config->spi_host_id, &spi_devcfg, &spi->hdl) == ESP_OK,
                      NULL, err, TAG, "adding device to SPI host #%d failed", ch390_config->spi_host_id + 1);

    /* create mutex */
    spi->lock = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(spi->lock, NULL, err, TAG, "create lock failed");

    ret = spi;
    return ret;
err:
    if (spi) {
        if (spi->lock) {
            vSemaphoreDelete(spi->lock);
        }
        free(spi);
    }
    return ret;
}

static esp_err_t ch390_spi_deinit(void *spi_ctx)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_bus_remove_device(spi->hdl);
    vSemaphoreDelete(spi->lock);

    free(spi);
    return ret;
}

static inline bool ch390_spi_lock(eth_spi_info_t *spi)
{
    return xSemaphoreTake(spi->lock, pdMS_TO_TICKS(CH390_SPI_LOCK_TIMEOUT_MS)) == pdTRUE;
}

static inline bool ch390_spi_unlock(eth_spi_info_t *spi)
{
    return xSemaphoreGive(spi->lock) == pdTRUE;
}

static esp_err_t ch390_spi_write(void *spi_ctx, uint32_t cmd, uint32_t addr, const void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .tx_buffer = value
    };
    if (ch390_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        ch390_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    return ret;
}

static esp_err_t ch390_spi_read(void *spi_ctx, uint32_t cmd, uint32_t addr, void *value, uint32_t len)
{
    esp_err_t ret = ESP_OK;
    eth_spi_info_t *spi = (eth_spi_info_t *)spi_ctx;

    spi_transaction_t trans = {
        .flags = len <= 4 ? SPI_TRANS_USE_RXDATA : 0, // use direct reads for registers to prevent overwrites by 4-byte boundary writes
        .cmd = cmd,
        .addr = addr,
        .length = 8 * len,
        .rx_buffer = value
    };
    if (ch390_spi_lock(spi)) {
        if (spi_device_polling_transmit(spi->hdl, &trans) != ESP_OK) {
            ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
            ret = ESP_FAIL;
        }
        ch390_spi_unlock(spi);
    } else {
        ret = ESP_ERR_TIMEOUT;
    }
    if ((trans.flags & SPI_TRANS_USE_RXDATA) && len <= 4) {
        memcpy(value, trans.rx_data, len);  // copy register values to output
    }
    return ret;
}

/**
 * @brief write value to ch390 internal register
 */
static esp_err_t ch390_register_write(emac_ch390_t *emac, uint8_t reg_addr, uint8_t value)
{
    return emac->spi.write(emac->spi.ctx, CH390_SPI_WR, reg_addr, &value, 1);
}

/**
 * @brief read value from ch390 internal register
 */
static esp_err_t ch390_register_read(emac_ch390_t *emac, uint8_t reg_addr, uint8_t *value)
{
    return emac->spi.read(emac->spi.ctx, CH390_SPI_RD, reg_addr, value, 1);
}

/**
 * @brief write buffer to ch390 internal memory
 */
static esp_err_t ch390_memory_write(emac_ch390_t *emac, uint8_t *buffer, uint32_t len)
{
    return emac->spi.write(emac->spi.ctx, CH390_SPI_WR, CH390_MWCMD, buffer, len);
}

/**
 * @brief read buffer from ch390 internal memory
 */
static esp_err_t ch390_memory_read(emac_ch390_t *emac, uint8_t *buffer, uint32_t len)
{
    return emac->spi.read(emac->spi.ctx, CH390_SPI_RD, CH390_MRCMD, buffer, len);
}

/**
 * @brief peek buffer from ch390 internal memory (without internal cursor moved)
 */
//Note: memory peek not working properly, so it's not used
static esp_err_t ch390_memory_peek(emac_ch390_t *emac, uint8_t *buffer, uint32_t len)
{
    return emac->spi.read(emac->spi.ctx, CH390_SPI_RD, CH390_MRCMDX1, buffer, len);
}

/**
 * @brief read mac address from internal registers
 */
static esp_err_t ch390_get_mac_addr(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < 6; i++) {
        ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_PAR + i, &emac->addr[i]), err, TAG, "read PAR failed");
    }
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief set new mac address to internal registers
 */
static esp_err_t ch390_set_mac_addr(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < 6; i++) {
        ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_PAR + i, emac->addr[i]), err, TAG, "write PAR failed");
    }
    ESP_LOGD(TAG, "MAC address set to %02x:%02x:%02x:%02x:%02x:%02x",
             emac->addr[0], emac->addr[1], emac->addr[2], emac->addr[3], emac->addr[4], emac->addr[5]);
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief clear multicast hash table
 */
static esp_err_t ch390_clear_multicast_table(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* broadcast packets controlled by MAC registers 16h~1Dh */
    for (int i = 0; i < 8; i++) {
        ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_MAR + i, 0x00), err, TAG, "write MAR failed");
    }
    ESP_LOGD(TAG, "multicast table cleared");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief software reset ch390 internal register
 */
static esp_err_t ch390_reset(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGD(TAG, "software resetting");
    /* software reset */
    uint8_t ncr = NCR_RST;
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_NCR, ncr), err, TAG, "write NCR failed");
    uint32_t to = 0;
    for (to = 0; to < emac->sw_reset_timeout_ms / 10; to++) {
        ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_NCR, &ncr), err, TAG, "read NCR failed");
        if (!(ncr & NCR_RST)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_GOTO_ON_FALSE(to < emac->sw_reset_timeout_ms / 10, ESP_ERR_TIMEOUT, err, TAG, "reset timeout");
    vTaskDelay(pdMS_TO_TICKS(10));
    /* power on phy */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_GPR, 0x00), err, TAG, "write GPR failed");
    /* mac and phy register won't be accesable within at least 1ms */
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGD(TAG, "phy powered on");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief verify ch390 chip ID
 */
static esp_err_t ch390_verify_id(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint8_t id[2];
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_VIDL, &id[0]), err, TAG, "read VIDL failed");
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_VIDH, &id[1]), err, TAG, "read VIDH failed");
    ESP_LOGD(TAG, "current chip ID: %02x%02x", id[1], id[0]);
    ESP_GOTO_ON_FALSE(0x1c == id[1] && 0x00 == id[0], ESP_ERR_INVALID_VERSION, err, TAG, "wrong Vendor ID");
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_PIDL, &id[0]), err, TAG, "read PIDL failed");
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_PIDH, &id[1]), err, TAG, "read PIDH failed");
    ESP_LOGD(TAG, "current Product ID: %02x%02x", id[1], id[0]);
    ESP_GOTO_ON_FALSE(0x91 == id[1] && 0x51 == id[0], ESP_ERR_INVALID_VERSION, err, TAG, "wrong Product ID");
    ESP_LOGD(TAG, "ch390 chip ID verified");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief default setup for ch390 internal registers
 */
static esp_err_t ch390_setup_default(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    /* disable wakeup */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_NCR, 0x00), err, TAG, "write NCR failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_WCR, 0x00), err, TAG, "write WCR failed");
    ESP_LOGD(TAG, "wakeup disabled");
    /* stop transmitting, enable appending pad, crc for packets */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_TCR, 0x00), err, TAG, "write TCR failed");
    ESP_LOGD(TAG, "clear TCR, stop tx");
    /* stop receiving, no promiscuous mode, no runt packet(size < 64bytes), receive all multicast packets */
    /* discard long packet(size > 1522bytes) and crc error packet, enable watchdog */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_RCR, RCR_DIS_CRC | RCR_ALL), err, TAG, "write RCR failed");
    ESP_LOGD(TAG, "reset RCR, discard CRC error packet, receive all multicast packets");
    /* interrupt pin config: push-pull output, active high */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_INTCR, 0x00), err, TAG, "write INTCR failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_INTCKCR, 0x00), err, TAG, "write INTCKCR failed");
    /* clear network status: wakeup event, tx complete */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END), err, TAG, "write NSR failed");
    /* BIT7 = 1, choose led mode 1 */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_TCR2, 0x80), err, TAG, "write TCR2 failed");
    ESP_LOGD(TAG, "reset TCR2, choose led mode 1");
    /* generate checksum for UDP, TCP and IPv4 packets */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_TCSCR, TCSCR_IPCSE | TCSCR_TCPCSE | TCSCR_UDPCSE), err, TAG, "write TCSCR failed");
    ESP_LOGD(TAG, "reset TCSCR, generate checksum for UDP, TCP and IPv4 packets");
    /* disable check sum for receive packets */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_RCSCSR, 0x00), err, TAG, "write RCSCSR failed");
    ESP_LOGD(TAG, "reset RCSCSR, checksum receive/drop packets");
    ESP_LOGD(TAG, "reset INTCR, active high");
    /* no length limitation for rx packets */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_RLENCR, 0x00), err, TAG, "write RLENCR failed");
    ESP_LOGD(TAG, "reset RLENCR, no length limitation for rx packets");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_enable_flow_ctrl(emac_ch390_t *emac, bool enable)
{
    esp_err_t ret = ESP_OK;
    if (enable) {
        /* send jam pattern (duration time = 1.15ms) when rx free space < 3k bytes */
        ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_BPTR, 0x3F), err, TAG, "write BPTR failed");
        /* flow control: high water threshold = 3k bytes, low water threshold = 8k bytes */
        ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_FCTR, FCTR_HWOT(3) | FCTR_LWOT(8)), err, TAG, "write FCTR failed");
        /* enable flow control */
        ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_FCR, FCR_FLOW_ENABLE), err, TAG, "write FCR failed");
    } else {
        /* disable flow control */
        ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_FCR, 0), err, TAG, "write FCR failed");
    }
    ESP_LOGD(TAG, "flow control %s", enable ? "enabled" : "disabled");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief start ch390: enable interrupt and start receive
 */
static esp_err_t emac_ch390_start(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    /* reset tx and rx memory pointer */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_MPTRCR, MPTRCR_RST_RX | MPTRCR_RST_TX), err, TAG, "write MPTRCR failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_MRRH, 0x0c), err, TAG, "write MPTRCR failed");
    ESP_LOGD(TAG, "rx/tx memory pointer reset");
    /* clear interrupt status */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_ISR, ISR_CLR_STATUS), err, TAG, "write ISR failed");
    ESP_LOGD(TAG, "interrupt status cleared");
    /* enable only Rx related interrupts as others are processed synchronously */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_IMR, IMR_PAR | IMR_PRI | IMR_ROI | ISR_ROO), err, TAG, "write IMR failed");
    ESP_LOGD(TAG, "interrupt enabled");
    /* enable rx */
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_RCR, &rcr), err, TAG, "read RCR failed");
    rcr |= RCR_RXEN;
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_RCR, rcr), err, TAG, "write RCR failed");
    ESP_LOGD(TAG, "rx enabled");
    ESP_LOGD(TAG, "ch390 emac started");
    return ESP_OK;
err:
    return ret;
}

/**
 * @brief stop ch390: disable interrupt and stop receive
 */
static esp_err_t emac_ch390_stop(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    /* disable interrupt */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_IMR, 0x00), err, TAG, "write IMR failed");
    ESP_LOGD(TAG, "interrupt disabled");
    /* disable rx */
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_RCR, &rcr), err, TAG, "read RCR failed");
    rcr &= ~RCR_RXEN;
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_RCR, rcr), err, TAG, "write RCR failed");
    ESP_LOGD(TAG, "rx disabled");
    ESP_LOGD(TAG, "ch390 emac stopped");
    return ESP_OK;
err:
    return ret;
}

IRAM_ATTR static void ch390_isr_handler(void *arg)
{
    emac_ch390_t *emac = (emac_ch390_t *)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    /* notify ch390 task */
    vTaskNotifyGiveFromISR(emac->rx_task_hdl, &high_task_wakeup);
    if (high_task_wakeup != pdFALSE) {
        portYIELD_FROM_ISR();
    }
}

static void ch390_poll_timer(void *arg)
{
    emac_ch390_t *emac = (emac_ch390_t *)arg;
    xTaskNotifyGive(emac->rx_task_hdl);
}

static esp_err_t emac_ch390_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *eth)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(eth, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac's mediator to null");
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    emac->eth = eth;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_write_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    /* check if phy access is in progress */
    uint8_t epcr = 0;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_EPCR, &epcr), err, TAG, "read EPCR failed");
    ESP_GOTO_ON_FALSE(!(epcr & EPCR_ERRE), ESP_ERR_INVALID_STATE, err, TAG, "phy is busy");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_EPAR, (uint8_t)(((phy_addr << 6) & 0xFF) | phy_reg)), err, TAG, "write EPAR failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_EPDRL, (uint8_t)(reg_value & 0xFF)), err, TAG, "write EPDRL failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_EPDRH, (uint8_t)((reg_value >> 8) & 0xFF)), err, TAG, "write EPDRH failed");
    /* select PHY and select write operation */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_EPCR, EPCR_EPOS | EPCR_ERPRW), err, TAG, "write EPCR failed");
    /* polling the busy flag */
    uint32_t to = 0;
    do {
        esp_rom_delay_us(100);
        ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_EPCR, &epcr), err, TAG, "read EPCR failed");
        to += 100;
    } while ((epcr & EPCR_ERRE) && to < CH390_PHY_OPERATION_TIMEOUT_US);
    ESP_GOTO_ON_FALSE(!(epcr & EPCR_ERRE), ESP_ERR_TIMEOUT, err, TAG, "phy is busy");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_read_phy_reg(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(reg_value, ESP_ERR_INVALID_ARG, err, TAG, "can't set reg_value to null");
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    /* check if phy access is in progress */
    uint8_t epcr = 0;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_EPCR, &epcr), err, TAG, "read EPCR failed");
    ESP_GOTO_ON_FALSE(!(epcr & 0x01), ESP_ERR_INVALID_STATE, err, TAG, "phy is busy");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_EPAR, (uint8_t)(((phy_addr << 6) & 0xFF) | phy_reg)), err, TAG, "write EPAR failed");
    /* Select PHY and select read operation */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_EPCR, 0x0C), err, TAG, "write EPCR failed");
    /* polling the busy flag */
    uint32_t to = 0;
    do {
        esp_rom_delay_us(100);
        ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_EPCR, &epcr), err, TAG, "read EPCR failed");
        to += 100;
    } while ((epcr & EPCR_ERRE) && to < CH390_PHY_OPERATION_TIMEOUT_US);
    ESP_GOTO_ON_FALSE(!(epcr & EPCR_ERRE), ESP_ERR_TIMEOUT, err, TAG, "phy is busy");
    uint8_t value_h = 0;
    uint8_t value_l = 0;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_EPDRH, &value_h), err, TAG, "read EPDRH failed");
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_EPDRL, &value_l), err, TAG, "read EPDRL failed");
    *reg_value = (value_h << 8) | value_l;
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    memcpy(emac->addr, addr, 6);
    ESP_GOTO_ON_ERROR(ch390_set_mac_addr(emac), err, TAG, "set mac address failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(addr, ESP_ERR_INVALID_ARG, err, TAG, "can't set mac addr to null");
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    memcpy(addr, emac->addr, 6);
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_set_link(esp_eth_mac_t *mac, eth_link_t link)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    switch (link) {
    case ETH_LINK_UP:
        ESP_GOTO_ON_ERROR(mac->start(mac), err, TAG, "ch390 start failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_start_periodic(emac->poll_timer, emac->poll_period_ms * 1000),
                              err, TAG, "start poll timer failed");
        }
        break;
    case ETH_LINK_DOWN:
        ESP_GOTO_ON_ERROR(mac->stop(mac), err, TAG, "ch390 stop failed");
        if (emac->poll_timer) {
            ESP_GOTO_ON_ERROR(esp_timer_stop(emac->poll_timer),
                              err, TAG, "stop poll timer failed");
        }
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown link status");
        break;
    }
    ESP_LOGD(TAG, "link status set to %s", link == ETH_LINK_UP ? "up" : "down");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_set_speed(esp_eth_mac_t *mac, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    switch (speed) {
    case ETH_SPEED_10M:
        ESP_LOGD(TAG, "working in 10Mbps");
        break;
    case ETH_SPEED_100M:
        ESP_LOGD(TAG, "working in 100Mbps");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown speed");
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_set_duplex(esp_eth_mac_t *mac, eth_duplex_t duplex)
{
    esp_err_t ret = ESP_OK;
    switch (duplex) {
    case ETH_DUPLEX_HALF:
        ESP_LOGD(TAG, "working in half duplex");
        break;
    case ETH_DUPLEX_FULL:
        ESP_LOGD(TAG, "working in full duplex");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unknown duplex");
        break;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_set_promiscuous(esp_eth_mac_t *mac, bool enable)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    uint8_t rcr = 0;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_RCR, &rcr), err, TAG, "read RCR failed");
    if (enable) {
        rcr |= RCR_PRMSC;
    } else {
        rcr &= ~RCR_PRMSC;
    }
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_RCR, rcr), err, TAG, "write RCR failed");
    ESP_LOGD(TAG, "promiscuous mode %s", enable ? "enabled" : "disabled");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t emac_ch390_enable_flow_ctrl(esp_eth_mac_t *mac, bool enable)
{
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    emac->flow_ctrl_enabled = enable;
    return ESP_OK;
}

static esp_err_t emac_ch390_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t ability)
{
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    // we want to enable flow control, and peer does support pause function
    // then configure the MAC layer to enable flow control feature
    if (emac->flow_ctrl_enabled && ability) {
        ch390_enable_flow_ctrl(emac, true);
    } else {
        ch390_enable_flow_ctrl(emac, false);
        ESP_LOGD(TAG, "Flow control not enabled for the link");
    }
    return ESP_OK;
}

static esp_err_t emac_ch390_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    /* Check if last transmit complete */
    uint8_t tcr = 0;

    ESP_GOTO_ON_FALSE(length <= ETH_MAX_PACKET_SIZE, ESP_ERR_INVALID_ARG, err,
                      TAG, "frame size is too big (actual %"PRIu32 ", maximum %u)", length, ETH_MAX_PACKET_SIZE);

    int64_t wait_time =  esp_timer_get_time();
    do {
        ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_TCR, &tcr), err, TAG, "read TCR failed");
    } while ((tcr & TCR_TXREQ) && ((esp_timer_get_time() - wait_time) < 100));

    if (tcr & TCR_TXREQ) {
        ESP_LOGE(TAG, "last transmit still in progress, cannot send.");
        return ESP_ERR_INVALID_STATE;
    }

    /* set tx length */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_TXPLL, length & 0xFF), err, TAG, "write TXPLL failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_TXPLH, (length >> 8) & 0xFF), err, TAG, "write TXPLH failed");
    /* copy data to tx memory */
    ESP_GOTO_ON_ERROR(ch390_memory_write(emac, buf, length), err, TAG, "write memory failed");
    /* issue tx polling command */
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_TCR, TCR_TXREQ), err, TAG, "write TCR failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_skip_recv_frame(emac_ch390_t *emac, uint16_t rx_length)
{
    esp_err_t ret = ESP_OK;
    uint8_t mrrh, mrrl;
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_MRRH, &mrrh), err, TAG, "read MDRAH failed");
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_MRRL, &mrrl), err, TAG, "read MDRAL failed");
    uint16_t addr = mrrh << 8 | mrrl;
    /* include 4B for header */
    addr += rx_length + CH390_RX_HDR_SIZE;
    if (addr > CH390_RX_MEM_MAX_SIZE) {
        addr = addr - CH390_RX_MEM_MAX_SIZE + CH390_RX_MEM_START_ADDR;
    }
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_MRRH, addr >> 8), err, TAG, "write MDRAH failed");
    ESP_GOTO_ON_ERROR(ch390_register_write(emac, CH390_MRRL, addr & 0xFF), err, TAG, "write MDRAL failed");
err:
    return ret;
}

static esp_err_t ch390_get_recv_byte_count(emac_ch390_t *emac, uint16_t *size)
{
    esp_err_t ret = ESP_OK;
    uint8_t rxbyte = 0;
    __attribute__((aligned(4))) ch390_rx_header_t header; // SPI driver needs the rx buffer 4 byte align

    *size = 0;
    /* dummy read, get the most updated data */
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
    /* rxbyte must be 0xFF, 0 or 1 */
    if (rxbyte & CH390_PKT_ERR) {
        ESP_LOGW(TAG, "receive status error: 0x%02x", rxbyte);
        ESP_GOTO_ON_ERROR(emac->parent.stop(&emac->parent), err, TAG, "stop ch390 failed");
        vTaskDelay(1);;
        ESP_GOTO_ON_ERROR(emac->parent.start(&emac->parent), err, TAG, "start ch390 failed");
        ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "reset rx fifo pointer");
    } else if (rxbyte & CH390_PKT_RDY) {
        ESP_GOTO_ON_ERROR(ch390_memory_peek(emac, (uint8_t *)&header, sizeof(header)), err, TAG, "peek rx header failed");
        uint16_t rx_len = header.length_low + (header.length_high << 8);
        /* RX Memory Overflow Error, we keep the packet */
        if (header.status & 0x01) {
            ESP_LOGW(TAG, "RX Memory Overflow");
        }
        if (header.status & 0x8f) {
            /* erroneous frames should not be forwarded by CH390, however, if it happens, just skip it */
            ch390_skip_recv_frame(emac, rx_len);
            ESP_GOTO_ON_FALSE(false, ESP_FAIL, err, TAG, "receive status error: 0x%x", header.status);
        }
        *size = rx_len;
    }
err:
    return ret;
}

static esp_err_t ch390_flush_recv_frame(emac_ch390_t *emac)
{
    esp_err_t ret = ESP_OK;
    uint16_t rx_len;
    ESP_GOTO_ON_ERROR(ch390_get_recv_byte_count(emac, &rx_len), err, TAG, "get rx frame length failed");
    ESP_GOTO_ON_ERROR(ch390_skip_recv_frame(emac, rx_len), err, TAG, "skipping frame in RX memory failed");
err:
    return ret;
}

static esp_err_t ch390_alloc_recv_buf(emac_ch390_t *emac, uint8_t **buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    uint16_t rx_len = 0;
    uint16_t byte_count;
    *buf = NULL;

    ESP_GOTO_ON_ERROR(ch390_get_recv_byte_count(emac, &byte_count), err, TAG, "get rx frame length failed");
    // silently return when no frame is waiting
    if (!byte_count) {
        goto err;
    }
    // do not include 4 bytes CRC at the end
    rx_len = byte_count - ETH_CRC_LEN;
    // frames larger than expected will be truncated
    uint16_t copy_len = rx_len > *length ? *length : rx_len;
    // runt frames are not forwarded, but check the length anyway since it could be corrupted at SPI bus
    ESP_GOTO_ON_FALSE(copy_len >= ETH_MIN_PACKET_SIZE - ETH_CRC_LEN, ESP_ERR_INVALID_SIZE, err, TAG, "invalid frame length %u", copy_len);
    *buf = malloc(copy_len);
    if (*buf != NULL) {
        ch390_auto_buf_info_t *buff_info = (ch390_auto_buf_info_t *)*buf;
        buff_info->copy_len = copy_len;
        buff_info->byte_cnt = byte_count;
    } else {
        ret = ESP_ERR_NO_MEM;
        goto err;
    }
err:
    *length = rx_len;
    return ret;
}

static esp_err_t emac_ch390_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    uint16_t rx_len = 0;
    uint8_t rxbyte;
    uint16_t copy_len = 0;
    uint16_t byte_count = 0;
    emac->packets_remain = false;

    if (*length != CH390_ETH_MAC_RX_BUF_SIZE_AUTO) {
        ESP_GOTO_ON_ERROR(ch390_get_recv_byte_count(emac, &byte_count), err, TAG, "get rx frame length failed");
        /* silently return when no frame is waiting */
        if (!byte_count) {
            goto err;
        }
        /* do not include 4 bytes CRC at the end */
        rx_len = byte_count - ETH_CRC_LEN;
        /* frames larger than expected will be truncated */
        copy_len = rx_len > *length ? *length : rx_len;
    } else {
        ch390_auto_buf_info_t *buff_info = (ch390_auto_buf_info_t *)buf;
        copy_len = buff_info->copy_len;
        byte_count = buff_info->byte_cnt;
    }

    byte_count += CH390_RX_HDR_SIZE;
    ESP_GOTO_ON_ERROR(ch390_memory_read(emac, emac->rx_buffer, byte_count), err, TAG, "read rx data failed");
    memcpy(buf, emac->rx_buffer + CH390_RX_HDR_SIZE, copy_len);
    *length = copy_len;

    /* dummy read, get the most updated data */
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
    /* check for remaing packets */
    ESP_GOTO_ON_ERROR(ch390_register_read(emac, CH390_MRCMDX, &rxbyte), err, TAG, "read MRCMDX failed");
    emac->packets_remain = rxbyte > 0;
    //TODO: DELETE
    printf("packets_remain: %d\n", emac->packets_remain);
    return ESP_OK;
err:
    *length = 0;
    return ret;
}

static esp_err_t emac_ch390_init(esp_eth_mac_t *mac)
{
    esp_err_t ret = ESP_OK;
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    if (emac->int_gpio_num >= 0) {
        esp_rom_gpio_pad_select_gpio(emac->int_gpio_num);
        gpio_set_direction(emac->int_gpio_num, GPIO_MODE_INPUT);
        gpio_set_pull_mode(emac->int_gpio_num, GPIO_PULLDOWN_ONLY);
        gpio_set_intr_type(emac->int_gpio_num, GPIO_INTR_POSEDGE);
        gpio_intr_enable(emac->int_gpio_num);
        gpio_isr_handler_add(emac->int_gpio_num, ch390_isr_handler, emac);
    }
    // Don't read/write CH390 registers within 10ms after power on
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LLINIT, NULL), err, TAG, "lowlevel init failed");
    /* reset ch390 */
    ESP_GOTO_ON_ERROR(ch390_reset(emac), err, TAG, "reset ch390 failed");
    /* verify chip id */
    ESP_GOTO_ON_ERROR(ch390_verify_id(emac), err, TAG, "vefiry chip ID failed");
    /* default setup of internal registers */
    ESP_GOTO_ON_ERROR(ch390_setup_default(emac), err, TAG, "ch390 default setup failed");
    /* clear multicast hash table */
    ESP_GOTO_ON_ERROR(ch390_clear_multicast_table(emac), err, TAG, "clear multicast table failed");
    /* get emac address from eeprom */
    ESP_GOTO_ON_ERROR(ch390_get_mac_addr(emac), err, TAG, "fetch ethernet mac address failed");
    return ESP_OK;
err:
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ret;
}

static esp_err_t emac_ch390_deinit(esp_eth_mac_t *mac)
{
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    esp_eth_mediator_t *eth = emac->eth;
    mac->stop(mac);
    if (emac->int_gpio_num >= 0) {
        gpio_isr_handler_remove(emac->int_gpio_num);
        gpio_reset_pin(emac->int_gpio_num);
    }
    if (emac->poll_timer && esp_timer_is_active(emac->poll_timer)) {
        esp_timer_stop(emac->poll_timer);
    }
    eth->on_state_changed(eth, ETH_STATE_DEINIT, NULL);
    return ESP_OK;
}

static void emac_ch390_task(void *arg)
{
    emac_ch390_t *emac = (emac_ch390_t *)arg;
    uint8_t status = 0;
    esp_err_t ret;
    while (1) {
        // check if the task receives any notification
        if (emac->int_gpio_num >= 0) {                                   // if in interrupt mode
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0 &&   // if no notification ...
                    gpio_get_level(emac->int_gpio_num) == 0) {               // ...and no interrupt asserted
                continue;                                                // -> just continue to check again
            }
        } else {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        /* clear interrupt status */
        ch390_register_read(emac, CH390_ISR, &status);
        ch390_register_write(emac, CH390_ISR, status);
        if (status & ISR_ROS) {
            printf("Receive overflow\r\n");
        }
        // Receive overflow counter overflow
        if (status & ISR_ROO) {
            printf("Overflow counter overflow\r\n");
        }

        if (status & ISR_LNKCHG) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            uint8_t nsr = 0;
            ch390_register_read(emac, CH390_NSR, &nsr);
            printf("Link status changed: %s\n", (nsr & NSR_LINKST) ? "up" : "down");
        }

        /* packet received */
        if (status & ISR_PR) {
            do {
                /* define max expected frame len */
                uint32_t frame_len = ETH_MAX_PACKET_SIZE;
                uint8_t *buffer;
                if ((ret = ch390_alloc_recv_buf(emac, &buffer, &frame_len)) == ESP_OK) {
                    if (buffer != NULL) {
                        /* we have memory to receive the frame of maximal size previously defined */
                        uint32_t buf_len = CH390_ETH_MAC_RX_BUF_SIZE_AUTO;
                        if (emac->parent.receive(&emac->parent, buffer, &buf_len) == ESP_OK) {
                            if (buf_len == 0) {
                                ch390_flush_recv_frame(emac);
                                free(buffer);
                            } else if (frame_len > buf_len) {
                                ESP_LOGE(TAG, "received frame was truncated");
                                free(buffer);
                            } else {
                                ESP_LOGD(TAG, "receive len=%" PRIu32, buf_len);
                                /* pass the buffer to stack (e.g. TCP/IP layer) */
                                emac->eth->stack_input(emac->eth, buffer, buf_len);
                            }
                        } else {
                            ESP_LOGE(TAG, "frame read from module failed");
                            ch390_flush_recv_frame(emac);
                            free(buffer);
                        }
                    } else if (frame_len) {
                        ESP_LOGE(TAG, "invalid combination of frame_len(%" PRIu32 ") and buffer pointer(%p)", frame_len, buffer);
                    }
                } else if (ret == ESP_ERR_NO_MEM) {
                    ESP_LOGE(TAG, "no mem for receive buffer");
                    ch390_flush_recv_frame(emac);
                } else {
                    ESP_LOGE(TAG, "unexpected error 0x%x", ret);
                }
            } while (emac->packets_remain);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t emac_ch390_del(esp_eth_mac_t *mac)
{
    emac_ch390_t *emac = __containerof(mac, emac_ch390_t, parent);
    if (emac->poll_timer) {
        esp_timer_delete(emac->poll_timer);
    }
    vTaskDelete(emac->rx_task_hdl);
    emac->spi.deinit(emac->spi.ctx);
    heap_caps_free(emac->rx_buffer);
    free(emac);
    return ESP_OK;
}

esp_eth_mac_t *esp_eth_mac_new_ch390(const eth_ch390_config_t *ch390_config, const eth_mac_config_t *mac_config)
{
    esp_eth_mac_t *ret = NULL;
    emac_ch390_t *emac = NULL;
    ESP_GOTO_ON_FALSE(ch390_config, NULL, err, TAG, "can't set ch390 specific config to null");
    ESP_GOTO_ON_FALSE(mac_config, NULL, err, TAG, "can't set mac config to null");
    ESP_GOTO_ON_FALSE((ch390_config->int_gpio_num >= 0) != (ch390_config->poll_period_ms > 0), NULL, err, TAG, "invalid configuration argument combination");
    emac = calloc(1, sizeof(emac_ch390_t));
    ESP_GOTO_ON_FALSE(emac, NULL, err, TAG, "calloc emac failed");
    /* bind methods and attributes */
    emac->sw_reset_timeout_ms = mac_config->sw_reset_timeout_ms;
    emac->int_gpio_num = ch390_config->int_gpio_num;
    emac->poll_period_ms = ch390_config->poll_period_ms;
    emac->parent.set_mediator = emac_ch390_set_mediator;
    emac->parent.init = emac_ch390_init;
    emac->parent.deinit = emac_ch390_deinit;
    emac->parent.start = emac_ch390_start;
    emac->parent.stop = emac_ch390_stop;
    emac->parent.del = emac_ch390_del;
    emac->parent.write_phy_reg = emac_ch390_write_phy_reg;
    emac->parent.read_phy_reg = emac_ch390_read_phy_reg;
    emac->parent.set_addr = emac_ch390_set_addr;
    emac->parent.get_addr = emac_ch390_get_addr;
    emac->parent.set_speed = emac_ch390_set_speed;
    emac->parent.set_duplex = emac_ch390_set_duplex;
    emac->parent.set_link = emac_ch390_set_link;
    emac->parent.set_promiscuous = emac_ch390_set_promiscuous;
    emac->parent.set_peer_pause_ability = emac_ch390_set_peer_pause_ability;
    emac->parent.enable_flow_ctrl = emac_ch390_enable_flow_ctrl;
    emac->parent.transmit = emac_ch390_transmit;
    emac->parent.receive = emac_ch390_receive;

    if (ch390_config->custom_spi_driver.init != NULL && ch390_config->custom_spi_driver.deinit != NULL
            && ch390_config->custom_spi_driver.read != NULL && ch390_config->custom_spi_driver.write != NULL) {
        ESP_LOGD(TAG, "Using user's custom SPI Driver");
        emac->spi.init = ch390_config->custom_spi_driver.init;
        emac->spi.deinit = ch390_config->custom_spi_driver.deinit;
        emac->spi.read = ch390_config->custom_spi_driver.read;
        emac->spi.write = ch390_config->custom_spi_driver.write;
        /* Custom SPI driver device init */
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(ch390_config->custom_spi_driver.config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    } else {
        ESP_LOGD(TAG, "Using default SPI Driver");
        emac->spi.init = ch390_spi_init;
        emac->spi.deinit = ch390_spi_deinit;
        emac->spi.read = ch390_spi_read;
        emac->spi.write = ch390_spi_write;
        /* SPI device init */
        ESP_GOTO_ON_FALSE((emac->spi.ctx = emac->spi.init(ch390_config)) != NULL, NULL, err, TAG, "SPI initialization failed");
    }

    /* create ch390 task */
    BaseType_t core_num = tskNO_AFFINITY;
    if (mac_config->flags & ETH_MAC_FLAG_PIN_TO_CORE) {
        core_num = esp_cpu_get_core_id();
    }
    BaseType_t xReturned = xTaskCreatePinnedToCore(emac_ch390_task, "ch390_tsk", mac_config->rx_task_stack_size, emac,
                           mac_config->rx_task_prio, &emac->rx_task_hdl, core_num);
    ESP_GOTO_ON_FALSE(xReturned == pdPASS, NULL, err, TAG, "create ch390 task failed");

    emac->rx_buffer = heap_caps_malloc(ETH_MAX_PACKET_SIZE + CH390_RX_HDR_SIZE, MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(emac->rx_buffer, NULL, err, TAG, "RX buffer allocation failed");

    if (emac->int_gpio_num < 0) {
        const esp_timer_create_args_t poll_timer_args = {
            .callback = ch390_poll_timer,
            .name = "emac_spi_poll_timer",
            .arg = emac,
            .skip_unhandled_events = true
        };
        ESP_GOTO_ON_FALSE(esp_timer_create(&poll_timer_args, &emac->poll_timer) == ESP_OK, NULL, err, TAG, "create poll timer failed");
    }

    return &(emac->parent);

err:
    if (emac) {
        if (emac->poll_timer) {
            esp_timer_delete(emac->poll_timer);
        }
        if (emac->rx_task_hdl) {
            vTaskDelete(emac->rx_task_hdl);
        }
        if (emac->spi.ctx) {
            emac->spi.deinit(emac->spi.ctx);
        }
        heap_caps_free(emac->rx_buffer);
        free(emac);
    }
    return ret;
}
