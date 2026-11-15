/*
 * ノード番号を NVS に置く (2026-11-15、GitHub issue #19)。宣言は dyne_node_id.h、決め方は dyne_node_id_core.c。
 *
 * 番号は起動時に1回だけ決め、起動中は変えない。ctl 命令 node_id で NVS を書き換えても、効くのは次の起動から。
 * 理由: 番号は MQTT の購読 (dyne/cfg/<id>・dyne/ctl/<id>)、接続時に登録する遺言 (LWT。切断時に broker が
 * dyne/node/<id>/status へ offline を貼る)、貼り置きの status、ワイヤ形式v2 の送信元、IMU の標本のトピックに
 * 使われている。走っている途中で変えると、古い番号の購読・遺言・貼り置きが残り、計算の途中なら送信元が変わる。
 * 再起動すれば全部が新しい番号でそろう。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "dyne_node.h"
#include "dyne_node_id.h"

static const char *TAG = "dyne-id";

#define NVS_NS "dyne"     /* dyne_ops.c と同じ名前空間 */
#define NVS_KEY "node_id" /* int32。無ければ MAC の番号 */

static int s_id = -1;
static dyne_id_src_t s_src = DYNE_ID_SRC_MAC;
static uint8_t s_mac[6];
static bool s_stored_has;  /* NVS に番号がある (書き換えた後の値) */
static int32_t s_stored;

static bool nvs_read_id(int32_t *v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_get_i32(h, NVS_KEY, v) == ESP_OK;
    nvs_close(h);
    return ok;
}

/* v < 0 なら消す */
static esp_err_t nvs_write_id(int32_t v)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    if (v >= 0) {
        e = nvs_set_i32(h, NVS_KEY, v);
    } else {
        e = nvs_erase_key(h, NVS_KEY);
        if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
    }
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e;
}

void dyne_node_id_init(void)
{
    if (s_id >= 0) return;
    esp_read_mac(s_mac, ESP_MAC_WIFI_STA);
    s_stored = -1;
    s_stored_has = nvs_read_id(&s_stored);
    s_id = dyne_node_id_select(CONFIG_DYNE_NODE_ID, s_stored_has, s_stored, s_mac, &s_src);
    printf("[dyne] node id %d source=%s (nvs=%ld, mac last byte=%u)\n", s_id, dyne_id_src_name(s_src),
           s_stored_has ? (long)s_stored : -1L, (unsigned)s_mac[5]);
    if (s_stored_has && s_src != DYNE_ID_SRC_NVS) {
        ESP_LOGW(TAG, "nvs node_id=%ld ignored (%s)", (long)s_stored,
                 s_src == DYNE_ID_SRC_CONFIG ? "CONFIG_DYNE_NODE_ID overrides" : "out of range");
    }
}

int dyne_node_id(void)
{
    if (s_id < 0) dyne_node_id_init();
    return s_id;
}

dyne_id_src_t dyne_node_id_src(void)
{
    return s_src;
}

/* 次の起動で選ばれる番号 */
static int next_id(dyne_id_src_t *src)
{
    return dyne_node_id_select(CONFIG_DYNE_NODE_ID, s_stored_has, s_stored, s_mac, src);
}

int dyne_node_id_fields(char *buf, int len)
{
    int nx = next_id(NULL);
    if (nx != dyne_node_id()) {
        return snprintf(buf, len, "\"id_src\":\"%s\",\"id_next\":%d", dyne_id_src_name(s_src), nx);
    }
    return snprintf(buf, len, "\"id_src\":\"%s\"", dyne_id_src_name(s_src));
}

static void pub_ack(const char *json)
{
    char sub[32];
    snprintf(sub, sizeof(sub), "node/%d/id/ack", dyne_node_id());
    ESP_LOGI(TAG, "id ack: %s", json);
    dyne_mqtt_pub(sub, json, 1, false, false);
}

bool dyne_node_id_on_ctl(const char *cmd, const cJSON *root)
{
    if (strcmp(cmd, "node_id") != 0) return false;
    if (s_id < 0) dyne_node_id_init();
    const cJSON *m = cJSON_GetObjectItem(root, "mac"), *i = cJSON_GetObjectItem(root, "id");
    int32_t want = -1;
    /* id が数でない (null・文字列など) ときは NaN を渡して「id が違う」にする */
    dyne_idcmd_t c = dyne_node_id_cmd(cJSON_IsString(m) ? m->valuestring : NULL, i != NULL,
                                      cJSON_IsNumber(i) ? i->valuedouble : NAN, s_mac, &want);
    if (c == DYNE_IDCMD_NOT_MINE) return true; /* 別の基板あて: 黙って無視 */

    char macs[18], json[256];
    snprintf(macs, sizeof(macs), "%02X:%02X:%02X:%02X:%02X:%02X", s_mac[0], s_mac[1], s_mac[2], s_mac[3],
             s_mac[4], s_mac[5]);
    const char *err = dyne_idcmd_err(c);
    bool write = c == DYNE_IDCMD_SET || c == DYNE_IDCMD_CLEAR;
    /* 計算の最中はフラッシュに書かない (書き込みの間はフラッシュ上のコードが止まり、刻みの時間を乱す) */
    if (!err && write && g_busy) err = "running";
    if (!err && write) {
        int32_t v = c == DYNE_IDCMD_SET ? want : -1;
        esp_err_t e = nvs_write_id(v);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "nvs write node_id: %s", esp_err_to_name(e));
            err = "nvs write";
        } else {
            s_stored_has = v >= 0;
            s_stored = v;
        }
    }
    if (err) {
        snprintf(json, sizeof(json), "{\"ok\":false,\"err\":\"%s\",\"mac\":\"%s\",\"id\":%d,\"src\":\"%s\"}", err,
                 macs, dyne_node_id(), dyne_id_src_name(s_src));
        pub_ack(json);
        return true;
    }
    dyne_id_src_t nsrc;
    int nx = next_id(&nsrc);
    bool reboot = write && nx != dyne_node_id() && cJSON_IsTrue(cJSON_GetObjectItem(root, "reboot"));
    const char *op = c == DYNE_IDCMD_SET ? "set" : c == DYNE_IDCMD_CLEAR ? "clear" : "query";
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"op\":\"%s\",\"mac\":\"%s\",\"id\":%d,\"src\":\"%s\",\"stored\":%ld,"
             "\"next\":%d,\"next_src\":\"%s\",\"reboot\":%s}",
             op, macs, dyne_node_id(), dyne_id_src_name(s_src), s_stored_has ? (long)s_stored : -1L, nx,
             dyne_id_src_name(nsrc), reboot ? "true" : "false");
    pub_ack(json);
    if (reboot) {
        /* main.c の reboot 命令と同じ手順 (古い番号の status に offline を貼ってから再起動) */
        char sub[32];
        snprintf(sub, sizeof(sub), "node/%d/status", dyne_node_id());
        dyne_mqtt_pub(sub, "{\"state\":\"offline\",\"reason\":\"node_id\"}", 1, true, false);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
    return true;
}
