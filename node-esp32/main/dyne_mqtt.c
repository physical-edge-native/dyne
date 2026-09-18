/*
 * Dyne node-esp32: MQTTクライアント層 (制御面第1段)
 *
 * 規約: docs/2026-09-14-mqtt-control-plane.md
 * この層に MQTT 依存を閉じ込める (第2段=zenoh-pico 移行時はこのファイルだけ
 * 差し替える。roadmap v3 3章原則7)。
 *
 * 応答義務 (規約 §3):
 *  - 接続時: status=online (retained) を発行、LWT (offline) を登録
 *  - dyne/cfg/<自ID> を購読し、受信を main.c (dyne_on_cfg) へ渡す
 *  - dyne/ctl/all と dyne/ctl/<自ID> を購読し、dyne_on_ctl へ渡す
 */
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "mqtt_client.h"

#include "dyne_node.h"

static const char *TAG = "dyne-mqtt";

static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected;
static char s_lwt_topic[48];

/* cfg はtransparent係数込みで数KBになりうる。分割到着に備えた受信バッファ */
#define CFG_BUF_MAX 8192
static char s_rxbuf[CFG_BUF_MAX];

bool dyne_mqtt_connected(void) { return s_connected; }

void dyne_mqtt_pub(const char *subtopic, const char *json, int qos, bool retain,
                   bool enqueue)
{
    if (!s_client) return;
    char topic[96];
    snprintf(topic, sizeof(topic), "dyne/%s", subtopic);
    if (enqueue) {
        /* BSPループ等から呼ぶ: ソケット送信を待たずアウトボックスに積む */
        esp_mqtt_client_enqueue(s_client, topic, json, 0, qos, retain, true);
    } else {
        esp_mqtt_client_publish(s_client, topic, json, 0, qos, retain);
    }
}

/* status の発行 (規約 §2.4)。extra_json_fields は先頭カンマなしの追記分 (NULL可) */
void dyne_mqtt_pub_status(dyne_state_t st, const char *extra)
{
    static const char *names[] = { "online", "configured", "running", "done", "err" };
    char sub[32], json[512];
    snprintf(sub, sizeof(sub), "node/%d/status", dyne_node_id());
    if (st == DYNE_ST_ONLINE) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        esp_netif_ip_info_t ip = { 0 };
        esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (nif) esp_netif_get_ip_info(nif, &ip);
        snprintf(json, sizeof(json),
                 "{\"state\":\"online\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                 "\"ip\":\"" IPSTR "\",\"fw\":\"%s\",\"heap\":%u%s%s}",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                 IP2STR(&ip.ip), DYNE_FW_NAME,
                 (unsigned)esp_get_free_heap_size(),
                 extra ? "," : "", extra ? extra : "");
    } else {
        snprintf(json, sizeof(json), "{\"state\":\"%s\",\"idx\":%d%s%s}",
                 names[st], g_cfg.valid ? g_cfg.idx : -1,
                 extra ? "," : "", extra ? extra : "");
    }
    dyne_mqtt_pub(sub, json, 1, true, false);
}

static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t ev = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED: {
        s_connected = true;
        char t[48];
        snprintf(t, sizeof(t), "dyne/cfg/%d", dyne_node_id());
        esp_mqtt_client_subscribe(s_client, t, 1);
        esp_mqtt_client_subscribe(s_client, "dyne/ctl/all", 1);
        snprintf(t, sizeof(t), "dyne/ctl/%d", dyne_node_id());
        esp_mqtt_client_subscribe(s_client, t, 1);
        ESP_LOGI(TAG, "connected; subscribed cfg/ctl for node %d", dyne_node_id());
        dyne_mqtt_pub_status(DYNE_ST_ONLINE, NULL);
        break;
    }
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "disconnected (auto-reconnect)");
        break;
    case MQTT_EVENT_DATA: {
        /* 分割到着の組み立て (cfg が受信バッファ長を超えた場合) */
        if (ev->total_data_len > CFG_BUF_MAX - 1) {
            ESP_LOGE(TAG, "payload too large: %d", ev->total_data_len);
            break;
        }
        memcpy(s_rxbuf + ev->current_data_offset, ev->data, ev->data_len);
        if (ev->current_data_offset + ev->data_len < ev->total_data_len) {
            break; /* 続きを待つ (topic は最初の断片にのみ載るが cfg/ctl の判別
                      には十分: 断片化するのは実質 cfg のみ) */
        }
        int len = ev->total_data_len;
        s_rxbuf[len] = 0;
        /* topic は最初の断片でのみ有効。断片化した大きなペイロード = cfg とみなす */
        if (ev->topic_len == 0 || strncmp(ev->topic, "dyne/cfg/", 9) == 0) {
            dyne_on_cfg(s_rxbuf, len);
        } else if (strncmp(ev->topic, "dyne/ctl/", 9) == 0) {
            dyne_on_ctl(s_rxbuf, len);
        }
        break;
    }
    default:
        break;
    }
}

void dyne_mqtt_start(void)
{
    snprintf(s_lwt_topic, sizeof(s_lwt_topic), "dyne/node/%d/status", dyne_node_id());
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_DYNE_BROKER_URI,
        .session.last_will = {
            /* 不意の離脱の通知 (規約 §2.4)。retained で最後の状態が残る */
            .topic = s_lwt_topic,
            .msg = "{\"state\":\"offline\",\"reason\":\"lwt\"}",
            .qos = 1,
            .retain = true,
        },
        .buffer.size = 8192,      /* transparent 係数込みの cfg を1イベントで受ける */
        .buffer.out_size = 4096,  /* field 等の観測発行用 */
    };
    s_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, on_mqtt, NULL);
    esp_mqtt_client_start(s_client);
}
