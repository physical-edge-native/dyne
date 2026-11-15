/*
 * Dyne node-esp32: RSSI収集 (発注7(iv))
 *
 * - AP RSSI: 接続中アクセスポイントの受信強度 (esp_wifi_sta_get_ap_info)。
 * - 近傍RSSI: プロミスキャス受信でデータフレームの送信元MACごとに受信強度を
 *   集計する (指数移動平均 + フレーム数)。データ面のUDPフレーム自体が測定源に
 *   なるため、計算参加ノード同士は追加通信なしで相互RSSIが得られる。
 *   MAC→ノードIDの対応は PC 側が status の mac で解決する (規約 §2.7)。
 *   Kconfig (DYNE_RSSI_SNIFF) で無効化可 (受信コールバックのCPUコストのため)。
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "dyne_timesync.h" /* 2026-11-11: 同期した時刻 */
#include "esp_wifi.h"

#include "dyne_node.h"

#define PEER_MAX 12

static struct {
    uint8_t mac[6];
    float rssi_ewma;
    uint32_t n;
    bool used;
} s_tab[PEER_MAX];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

#ifdef CONFIG_DYNE_RSSI_SNIFF
static void sniff_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_DATA) return;
    const wifi_promiscuous_pkt_t *p = buf;
    /* 802.11 ヘッダ: addr2 (送信元) はオフセット10 */
    const uint8_t *mac = p->payload + 10;
    int8_t rssi = p->rx_ctrl.rssi;
    portENTER_CRITICAL(&s_mux);
    int free_i = -1;
    for (int i = 0; i < PEER_MAX; i++) {
        if (s_tab[i].used && memcmp(s_tab[i].mac, mac, 6) == 0) {
            s_tab[i].rssi_ewma = 0.8f * s_tab[i].rssi_ewma + 0.2f * rssi;
            s_tab[i].n++;
            portEXIT_CRITICAL(&s_mux);
            return;
        }
        if (!s_tab[i].used && free_i < 0) free_i = i;
    }
    if (free_i >= 0) {
        memcpy(s_tab[free_i].mac, mac, 6);
        s_tab[free_i].rssi_ewma = rssi;
        s_tab[free_i].n = 1;
        s_tab[free_i].used = true;
    }
    portEXIT_CRITICAL(&s_mux);
}
#endif

#ifdef CONFIG_DYNE_RSSI_SNIFF
/* 実行時の縮退 (発注7検収回答(3)): rssi_period_ms=0 の設定注入でプロミスキャス
 * 受信そのものを止める (受信コールバックのCPUコストをゼロにする)。period>0 に
 * 戻せば再開する。ビルド時の全無効化は Kconfig (DYNE_RSSI_SNIFF)。 */
static void sniff_enable(bool on)
{
    static bool cur = true; /* dyne_rssi_start で有効化済み */
    if (on == cur) return;
    esp_wifi_set_promiscuous(on);
    cur = on;
}
#endif

static void rssi_task(void *arg)
{
    for (;;) {
        int period = g_cfg.valid ? g_cfg.rssi_period_ms : 10000;
#ifdef CONFIG_DYNE_RSSI_SNIFF
        sniff_enable(period > 0);
#endif
        if (period <= 0 || !dyne_mqtt_connected()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        wifi_ap_record_t ap;
        int ap_rssi = 0;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ap_rssi = ap.rssi;

        char json[640];
        int64_t t_local = esp_timer_get_time();
        int off = snprintf(json, sizeof(json), "{\"t_ms\":%lld,\"ts_us\":%lld,\"ap_rssi\":%d,\"peers\":[",
                           (long long)(t_local / 1000), (long long)dyne_time_at_local(t_local), ap_rssi);
        /* 表を読み出してリセット (周期ごとの観測数を n に持たせる) */
        portENTER_CRITICAL(&s_mux);
        int cnt = 0;
        for (int i = 0; i < PEER_MAX && off < (int)sizeof(json) - 64; i++) {
            if (!s_tab[i].used) continue;
            off += snprintf(json + off, sizeof(json) - off,
                            "%s{\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                            "\"rssi\":%.0f,\"n\":%u}",
                            cnt ? "," : "",
                            s_tab[i].mac[0], s_tab[i].mac[1], s_tab[i].mac[2],
                            s_tab[i].mac[3], s_tab[i].mac[4], s_tab[i].mac[5],
                            (double)s_tab[i].rssi_ewma, (unsigned)s_tab[i].n);
            s_tab[i].used = false;
            cnt++;
        }
        portEXIT_CRITICAL(&s_mux);
        snprintf(json + off, sizeof(json) - off, "]}");

        char sub[32];
        snprintf(sub, sizeof(sub), "node/%d/rssi", dyne_node_id());
        dyne_mqtt_pub(sub, json, 0, false, true);
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

void dyne_rssi_start(void)
{
#ifdef CONFIG_DYNE_RSSI_SNIFF
    wifi_promiscuous_filter_t f = { .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA };
    esp_wifi_set_promiscuous_filter(&f);
    esp_wifi_set_promiscuous_rx_cb(sniff_cb);
    esp_wifi_set_promiscuous(true);
#endif
    xTaskCreate(rssi_task, "dyne_rssi", 4096, NULL, 3, NULL);
}
