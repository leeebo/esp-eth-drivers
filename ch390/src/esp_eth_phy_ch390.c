/*
 * SPDX-FileCopyrightText: 2019-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_eth_phy_802_3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ch390.phy";


typedef struct {
    phy_802_3_t phy_802_3;
} phy_ch390_t;

static esp_err_t ch390_update_link_duplex_speed(phy_ch390_t *ch390)
{
    esp_err_t ret = ESP_OK;
    esp_eth_mediator_t *eth = ch390->phy_802_3.eth;
    uint32_t addr = ch390->phy_802_3.addr;
    eth_speed_t speed = ETH_SPEED_10M;
    eth_duplex_t duplex = ETH_DUPLEX_HALF;
    uint32_t peer_pause_ability = false;
    bmsr_reg_t bmsr;
    bmcr_reg_t bmcr;
    anlpar_reg_t anlpar;
    // BMSR is a latch low register
    // after power up, the first latched value must be 0, which means down
    // to speed up power up link speed, double read this register as a workaround
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)), err, TAG, "read BMSR failed");
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, addr, ETH_PHY_BMSR_REG_ADDR, &(bmsr.val)), err, TAG, "read BMSR failed");
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, addr, ETH_PHY_ANLPAR_REG_ADDR, &(anlpar.val)), err, TAG, "read ANLPAR failed");
    eth_link_t link = bmsr.link_status ? ETH_LINK_UP : ETH_LINK_DOWN;
    /* check if link status changed */
    if (ch390->phy_802_3.link_status != link) {
        ESP_LOGI(TAG, "Link Status: %s", link == ETH_LINK_UP ? "UP" : "DOWN");
        /* when link up, read negotiation result */
        if (link == ETH_LINK_UP) {
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)), err, TAG, "read BMCR failed");
            speed = bmcr.speed_select == 1 ? ETH_SPEED_100M : ETH_SPEED_10M;
            duplex = bmcr.duplex_mode == 1 ? ETH_DUPLEX_FULL : ETH_DUPLEX_HALF;
            ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_SPEED, (void *)speed), err, TAG, "change speed failed");
            ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_DUPLEX, (void *)duplex), err, TAG, "change duplex failed");
            /* if we're in duplex mode, and peer has the flow control ability */
            if (duplex == ETH_DUPLEX_FULL && anlpar.symmetric_pause) {
                peer_pause_ability = 1;
            } else {
                peer_pause_ability = 0;
            }
            ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_PAUSE, (void *)peer_pause_ability), err, TAG, "change pause ability failed");
        }
        ESP_GOTO_ON_ERROR(eth->on_state_changed(eth, ETH_STATE_LINK, (void *)link), err, TAG, "change link failed");
        ch390->phy_802_3.link_status = link;
    }
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_get_link(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    phy_ch390_t *ch390 = __containerof(esp_eth_phy_into_phy_802_3(phy), phy_ch390_t, phy_802_3);
    /* Update information about link, speed, duplex */
    ESP_GOTO_ON_ERROR(ch390_update_link_duplex_speed(ch390), err, TAG, "update link duplex speed failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_reset(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    phy_ch390_t *ch390 = __containerof(esp_eth_phy_into_phy_802_3(phy), phy_ch390_t, phy_802_3);
    uint32_t addr = ch390->phy_802_3.addr;
    ch390->phy_802_3.link_status = ETH_LINK_DOWN;
    esp_eth_mediator_t *eth = ch390->phy_802_3.eth;
    bmcr_reg_t bmcr = {.reset = 1};
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val), err, TAG, "write BMCR failed");
    /* Wait for reset complete */
    uint32_t to = 0;
    for (to = 0; to < ch390->phy_802_3.reset_timeout_ms / 10; to++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)), err, TAG, "read BMCR failed");
        if (!bmcr.reset) {
            break;
        }
    }
    ESP_GOTO_ON_FALSE(to < ch390->phy_802_3.reset_timeout_ms / 10, ESP_FAIL, err, TAG, "PHY reset timeout");
    /* set phy mode to auto */
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, addr, ETH_PHY_BMCR_REG_ADDR, 0x1000), err, TAG, "PHY reset BMCR timeout");
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, addr, ETH_PHY_ANAR_REG_ADDR, 0x01E1), err, TAG, "PHY reset ANAR timeout");
    ESP_LOGD(TAG, "reset complete");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_init(esp_eth_phy_t *phy)
{
    esp_err_t ret = ESP_OK;
    phy_802_3_t *phy_802_3 = esp_eth_phy_into_phy_802_3(phy);

    /* Basic PHY init */
    ESP_GOTO_ON_ERROR(esp_eth_phy_802_3_basic_phy_init(phy_802_3), err, TAG, "failed to init PHY");

    /* Check PHY ID */
    uint32_t oui;
    uint8_t model;
    ESP_GOTO_ON_ERROR(esp_eth_phy_802_3_read_oui(phy_802_3, &oui), err, TAG, "read OUI failed");
    ESP_LOGD(TAG, "current oui: 0x%06x", (unsigned int)oui);
    ESP_GOTO_ON_ERROR(esp_eth_phy_802_3_read_manufac_info(phy_802_3, &model, NULL), err, TAG, "read manufacturer's info failed");
    ESP_LOGD(TAG, "current model: 0x%02x", model);
    /* The actual OUI of CH390 is 0x1cdc64 */
    ESP_GOTO_ON_FALSE(oui == 0x1cdc64 && model == 0x01, ESP_FAIL, err, TAG, "wrong chip ID");
    ESP_GOTO_ON_ERROR(ch390_reset(phy), err, TAG, "reset phy failed");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_loopback(esp_eth_phy_t *phy, bool enable)
{
    esp_err_t ret = ESP_OK;
    phy_802_3_t *phy_802_3 = esp_eth_phy_into_phy_802_3(phy);
    esp_eth_mediator_t *eth = phy_802_3->eth;
    /* Set Loopback function */
    bmcr_reg_t bmcr;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, phy_802_3->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)), err, TAG, "read BMCR failed");
    if (enable) {
        bmcr.en_loopback = 1;
    } else {
        bmcr.en_loopback = 0;
    }
    ESP_GOTO_ON_ERROR(eth->phy_reg_write(eth, phy_802_3->addr, ETH_PHY_BMCR_REG_ADDR, bmcr.val), err, TAG, "write BMCR failed");
    ESP_LOGD(TAG, "loopback %s", enable ? "enabled" : "disabled");
    return ESP_OK;
err:
    return ret;
}

static esp_err_t ch390_set_speed(esp_eth_phy_t *phy, eth_speed_t speed)
{
    esp_err_t ret = ESP_OK;
    phy_802_3_t *phy_802_3 = esp_eth_phy_into_phy_802_3(phy);
    esp_eth_mediator_t *eth = phy_802_3->eth;

    /* Check if loopback is enabled, and if so, can it work with proposed speed or not */
    bmcr_reg_t bmcr;
    ESP_GOTO_ON_ERROR(eth->phy_reg_read(eth, phy_802_3->addr, ETH_PHY_BMCR_REG_ADDR, &(bmcr.val)), err, TAG, "read BMCR failed");
    if (bmcr.en_loopback) {
        ESP_GOTO_ON_FALSE(speed == ETH_SPEED_100M, ESP_ERR_INVALID_STATE, err, TAG, "Speed must be 100M for loopback operation");
    }
    ESP_LOGD(TAG, "set speed to %s", speed == ETH_SPEED_100M ? "100M" : "10M");
    return esp_eth_phy_802_3_set_speed(phy_802_3, speed);
err:
    return ret;
}

esp_eth_phy_t *esp_eth_phy_new_ch390(const eth_phy_config_t *config)
{
    esp_eth_phy_t *ret = NULL;
    phy_ch390_t *ch390 = calloc(1, sizeof(phy_ch390_t));
    ESP_GOTO_ON_FALSE(ch390, NULL, err, TAG, "calloc ch390 failed");
    ESP_GOTO_ON_FALSE(esp_eth_phy_802_3_obj_config_init(&ch390->phy_802_3, config) == ESP_OK,
                      NULL, err, TAG, "configuration initialization of PHY 802.3 failed");

    // redefine functions which need to be customized for sake of ch390
    ch390->phy_802_3.parent.init = ch390_init;
    ch390->phy_802_3.parent.reset = ch390_reset;
    ch390->phy_802_3.parent.get_link = ch390_get_link;
    ch390->phy_802_3.parent.loopback = ch390_loopback;
    ch390->phy_802_3.parent.set_speed = ch390_set_speed;

    return &ch390->phy_802_3.parent;
err:
    if (ch390 != NULL) {
        free(ch390);
    }
    return ret;
}
