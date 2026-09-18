/*
 * Dyne node-esp32: 設定注入駆動の汎用ノードファームウェア (発注7、2026-09-14)
 *
 * 全ノード同一ファームウェア。役割は MQTT 制御面からの設定注入で決まる
 * (roadmap v3 4章)。m2-esp32 (2台固定・コンパイル時役割) がベース。
 *
 * 構成:
 *  - 制御面: MQTT 第1段 (dyne_mqtt.c、規約 docs/2026-09-14-mqtt-control-plane.md)。
 *    ブートストラップは Wi-Fi SSID/パスワード・ブローカーURI・ノードIDのみ
 *    (menuconfig)。それ以外の設定 (担当区間、境界条件キー、界面の結線、
 *    シード、モード) はすべて dyne/cfg/<id> で注入する。
 *  - データ面: 自前40バイト意味論 + UDP直送 (M2から継続。MQTTを通らない)。
 *    BSPステップ同期、送信控え+応答再送、再送バックオフ、終了時ドレイン。
 *  - 計算: 実機と同一の WASM アクター (heat1d_actor.wasm、F1の4キー対応版) を
 *    WAMR (Classic interpreter) で実行。
 *  - F1境界条件 (発注7(iii)): dirichlet(値つき)/zero_grad/linear_extrap は
 *    bcコードの注入のみ。transparent は指数和係数をPCが計算して設定注入し
 *    (規約 §2.1)、ファームは dyne_transparent_coef へ流し込むだけ。
 *  - 観測: 温度場・界面残差・リアルタイム率 (発注7(v))・センサ (sensors.c)・
 *    RSSI (rssi.c) を MQTT で発行。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "cJSON.h"

#include "wasm_export.h"
#include "dyne_node.h"

static const char *TAG = "dyne-node";

/* ==== 埋め込みWASM (F1の4キー対応版 heat1d_actor) ==== */
extern const uint8_t wasm_start[] asm("_binary_heat1d_actor_wasm_start");
extern const uint8_t wasm_end[]   asm("_binary_heat1d_actor_wasm_end");

/* ==== グローバル ==== */
dyne_cfg_t g_cfg;
volatile bool g_start_req, g_stop_req, g_report_req;

#define TIMEOUT_US     100000 /* 返事待ちの上限 (M0と同じ100ms) */
#define RESEND_MIN_US  20000  /* 再送バックオフ (M1知見4) */
#define RECV_SLICE_MS  20
#define DRAIN_MS       3000

int dyne_node_id(void)
{
#if CONFIG_DYNE_NODE_ID >= 0
    return CONFIG_DYNE_NODE_ID;
#else
    /* 未設定ならMAC末尾バイト (仮ID)。本運用では menuconfig で明示すること */
    static int id = -1;
    if (id < 0) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        id = mac[5];
    }
    return id;
#endif
}

/* ==== ワイヤ形式 (dyne-wire と同一・40バイト) ==== */
uint32_t dyne_crc32(const uint8_t *d, size_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) {
            uint32_t m = -(c & 1u);
            c = (c >> 1) ^ (0xEDB88320u & m);
        }
    }
    return ~c;
}

void dyne_wire_encode(uint8_t *b, uint16_t iface, uint64_t step, uint32_t seq,
                      double value, double grad)
{
    const uint32_t magic = 0x44594E30u;
    memcpy(b + 0, &magic, 4);
    memcpy(b + 4, &iface, 2);
    memset(b + 6, 0, 2);
    memcpy(b + 8, &step, 8);
    memcpy(b + 16, &seq, 4);
    memcpy(b + 20, &value, 8);
    memcpy(b + 28, &grad, 8);
    uint32_t crc = dyne_crc32(b, 36);
    memcpy(b + 36, &crc, 4);
}

bool dyne_wire_decode(const uint8_t *b, int len, uint16_t *iface,
                      uint64_t *step, double *value, double *grad)
{
    if (len != DYNE_WIRE_LEN) return false;
    uint32_t magic, crc;
    memcpy(&magic, b + 0, 4);
    if (magic != 0x44594E30u) return false;
    memcpy(&crc, b + 36, 4);
    if (dyne_crc32(b, 36) != crc) return false;
    memcpy(iface, b + 4, 2);
    memcpy(step, b + 8, 8);
    memcpy(value, b + 20, 8);
    memcpy(grad, b + 28, 8);
    return true;
}

/* ==== Wi-Fi (STA。APはPC側=ホットスポットまたはルーター) ==== */
static EventGroupHandle_t s_ev;
#define EV_GOT_IP BIT0

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "wifi disconnected, retrying...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_ev, EV_GOT_IP);
    }
}

static void wifi_up(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_ev = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL));
    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, CONFIG_DYNE_WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, CONFIG_DYNE_WIFI_PASS, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "connecting to '%s'...", CONFIG_DYNE_WIFI_SSID);
    xEventGroupWaitBits(s_ev, EV_GOT_IP, pdFALSE, pdTRUE, portMAX_DELAY);
    /* 省電力オフ (M2知見: 有効のままだと受信が間欠になり遅延が暴れる) */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "got IP");
}

/* ==== 設定注入の解釈 (規約 §2.1〜2.2) ==== */

static int bc_key_to_code(const cJSON *bcobj, double *value, char *err, size_t errlen)
{
    const cJSON *k = cJSON_GetObjectItem(bcobj, "key");
    if (!cJSON_IsString(k)) { snprintf(err, errlen, "bc key missing"); return -1; }
    const char *s = k->valuestring;
    *value = 0.0;
    if (strcmp(s, "dirichlet") == 0) {
        const cJSON *v = cJSON_GetObjectItem(bcobj, "value");
        if (cJSON_IsNumber(v)) *value = v->valuedouble;
        return DYNE_BC_DIRICHLET;
    }
    if (strcmp(s, "interface") == 0) return DYNE_BC_INTERFACE;
    if (strcmp(s, "zero_grad") == 0) return DYNE_BC_ZERO_GRAD;
    if (strcmp(s, "linear_extrap") == 0) return DYNE_BC_LINEAR_EXTRAP;
    if (strcmp(s, "transparent") == 0) return DYNE_BC_TRANSPARENT;
    snprintf(err, errlen, "bad bc key: %s", s);
    return -1;
}

static bool parse_transparent(const cJSON *bcobj, dyne_cfg_t *c, char *err, size_t errlen)
{
    const cJSON *m = cJSON_GetObjectItem(bcobj, "m");
    const cJSON *de = cJSON_GetObjectItem(bcobj, "decay");
    const cJSON *ha = cJSON_GetObjectItem(bcobj, "half");
    const cJSON *we = cJSON_GetObjectItem(bcobj, "weight");
    if (!cJSON_IsNumber(m) || !cJSON_IsArray(de) || !cJSON_IsArray(ha) || !cJSON_IsArray(we)) {
        snprintf(err, errlen, "transparent coef missing");
        return false;
    }
    int mm = m->valueint;
    if (mm < 2 || mm > DYNE_SOE_MAX || cJSON_GetArraySize(de) != mm
        || cJSON_GetArraySize(ha) != mm || cJSON_GetArraySize(we) != mm) {
        snprintf(err, errlen, "transparent coef size");
        return false;
    }
    for (int j = 0; j < mm; j++) {
        c->t_decay[j] = cJSON_GetArrayItem(de, j)->valuedouble;
        c->t_half[j] = cJSON_GetArrayItem(ha, j)->valuedouble;
        c->t_weight[j] = cJSON_GetArrayItem(we, j)->valuedouble;
    }
    c->t_m = mm;
    return true;
}

static bool parse_cfg(const char *payload, int len, dyne_cfg_t *c, char *err, size_t errlen)
{
    memset(c, 0, sizeof(*c));
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) { snprintf(err, errlen, "json parse"); return false; }
    bool ok = false;
    do {
        const cJSON *cl = cJSON_GetObjectItem(root, "cluster");
        const cJSON *nd = cJSON_GetObjectItem(root, "node");
        const cJSON *bc = cJSON_GetObjectItem(root, "bc");
        const cJSON *ifs = cJSON_GetObjectItem(root, "ifaces");
        const cJSON *mo = cJSON_GetObjectItem(root, "mode");
        if (!cl || !nd || !bc) { snprintf(err, errlen, "cluster/node/bc missing"); break; }
        c->n_nodes = cJSON_GetObjectItem(cl, "n_nodes") ? cJSON_GetObjectItem(cl, "n_nodes")->valueint : 0;
        c->points = cJSON_GetObjectItem(cl, "points") ? cJSON_GetObjectItem(cl, "points")->valueint : 0;
        c->alpha = cJSON_GetObjectItem(cl, "alpha") ? cJSON_GetObjectItem(cl, "alpha")->valuedouble : 1.0;
        c->dt = cJSON_GetObjectItem(cl, "dt") ? cJSON_GetObjectItem(cl, "dt")->valuedouble : 1e-4;
        c->steps = cJSON_GetObjectItem(cl, "steps") ? (uint32_t)cJSON_GetObjectItem(cl, "steps")->valuedouble : 0;
        c->seed = cJSON_GetObjectItem(cl, "seed") ? (uint32_t)cJSON_GetObjectItem(cl, "seed")->valuedouble : 42;
        c->idx = cJSON_GetObjectItem(nd, "idx") ? cJSON_GetObjectItem(nd, "idx")->valueint : -1;
        c->n = cJSON_GetObjectItem(nd, "n") ? cJSON_GetObjectItem(nd, "n")->valueint : 0;
        c->x0 = cJSON_GetObjectItem(nd, "x0") ? cJSON_GetObjectItem(nd, "x0")->valuedouble : 0.0;
        c->x1 = cJSON_GetObjectItem(nd, "x1") ? cJSON_GetObjectItem(nd, "x1")->valuedouble : 1.0;
        if (c->n < 3 || c->steps == 0) { snprintf(err, errlen, "bad n/steps"); break; }

        const cJSON *bl = cJSON_GetObjectItem(bc, "left");
        const cJSON *br = cJSON_GetObjectItem(bc, "right");
        if (!bl || !br) { snprintf(err, errlen, "bc left/right missing"); break; }
        c->bc_left = bc_key_to_code(bl, &c->bc_val_left, err, errlen);
        if (c->bc_left < 0) break;
        c->bc_right = bc_key_to_code(br, &c->bc_val_right, err, errlen);
        if (c->bc_right < 0) break;
        if (c->bc_left == DYNE_BC_TRANSPARENT && !parse_transparent(bl, c, err, errlen)) break;
        if (c->bc_right == DYNE_BC_TRANSPARENT && !parse_transparent(br, c, err, errlen)) break;

        c->n_ifaces = 0;
        if (cJSON_IsArray(ifs)) {
            const cJSON *e;
            cJSON_ArrayForEach(e, ifs) {
                if (c->n_ifaces >= DYNE_MAX_IFACES) break;
                const cJSON *id = cJSON_GetObjectItem(e, "id");
                const cJSON *fc = cJSON_GetObjectItem(e, "face");
                const cJSON *pr = cJSON_GetObjectItem(e, "peer");
                if (!cJSON_IsNumber(id) || !cJSON_IsString(fc) || !cJSON_IsString(pr)) {
                    snprintf(err, errlen, "iface entry"); goto fail;
                }
                char ip[24];
                int port = 0;
                if (sscanf(pr->valuestring, "%23[^:]:%d", ip, &port) != 2) {
                    snprintf(err, errlen, "iface peer '%s'", pr->valuestring); goto fail;
                }
                c->ifaces[c->n_ifaces].id = id->valueint;
                c->ifaces[c->n_ifaces].face = (strcmp(fc->valuestring, "right") == 0) ? 1 : 0;
                memset(&c->ifaces[c->n_ifaces].peer, 0, sizeof(struct sockaddr_in));
                c->ifaces[c->n_ifaces].peer.sin_family = AF_INET;
                c->ifaces[c->n_ifaces].peer.sin_port = htons(port);
                c->ifaces[c->n_ifaces].peer.sin_addr.s_addr = inet_addr(ip);
                c->n_ifaces++;
            }
        }
        /* 界面の面と bc=interface の整合を確認する */
        for (int i = 0; i < c->n_ifaces; i++) {
            int code = c->ifaces[i].face == 0 ? c->bc_left : c->bc_right;
            if (code != DYNE_BC_INTERFACE) {
                snprintf(err, errlen, "iface on non-interface bc face"); goto fail;
            }
        }

        c->role = 0;
        c->sensor = 0;
        c->sensor_period_ms = 5000;
        c->rssi_period_ms = 10000;
        c->field_every = 50;
        c->residual_every = 10;
        c->rt_every = 100;
        if (mo) {
            const cJSON *v;
            if ((v = cJSON_GetObjectItem(mo, "role")) && cJSON_IsString(v))
                c->role = strcmp(v->valuestring, "observe") == 0 ? 1 : 0;
            if ((v = cJSON_GetObjectItem(mo, "sensor")) && cJSON_IsString(v)) {
                if (strcmp(v->valuestring, "dht20") == 0) c->sensor = 1;
                else if (strcmp(v->valuestring, "bme280") == 0) c->sensor = 2;
            }
            if ((v = cJSON_GetObjectItem(mo, "sensor_period_ms"))) c->sensor_period_ms = v->valueint;
            if ((v = cJSON_GetObjectItem(mo, "rssi_period_ms"))) c->rssi_period_ms = v->valueint;
            if ((v = cJSON_GetObjectItem(mo, "field_every"))) c->field_every = v->valueint;
            if ((v = cJSON_GetObjectItem(mo, "residual_every"))) c->residual_every = v->valueint;
            if ((v = cJSON_GetObjectItem(mo, "rt_every"))) c->rt_every = v->valueint;
        }
        ok = true;
    } while (0);
fail:
    cJSON_Delete(root);
    c->valid = ok;
    return ok;
}

void dyne_on_cfg(const char *payload, int len)
{
    char err[96] = "";
    dyne_cfg_t tmp;
    char sub[32], json[160];
    snprintf(sub, sizeof(sub), "cfg/%d/ack", dyne_node_id());
    if (parse_cfg(payload, len, &tmp, err, sizeof(err))) {
        g_cfg = tmp; /* 受理 (startまで何度でも上書き可) */
        snprintf(json, sizeof(json), "{\"ok\":true,\"idx\":%d,\"crc32\":\"0x%08X\"}",
                 g_cfg.idx, (unsigned)dyne_crc32((const uint8_t *)payload, len));
        dyne_mqtt_pub(sub, json, 1, false, false);
        dyne_mqtt_pub_status(DYNE_ST_CONFIGURED, NULL);
        ESP_LOGI(TAG, "cfg accepted: idx=%d n=%d [%f,%f] bc=[%d,%d] ifaces=%d",
                 g_cfg.idx, g_cfg.n, g_cfg.x0, g_cfg.x1, g_cfg.bc_left,
                 g_cfg.bc_right, g_cfg.n_ifaces);
    } else {
        snprintf(json, sizeof(json), "{\"ok\":false,\"idx\":%d,\"err\":\"%s\"}",
                 dyne_node_id(), err);
        dyne_mqtt_pub(sub, json, 1, false, false);
        ESP_LOGE(TAG, "cfg rejected: %s", err);
    }
}

void dyne_on_ctl(const char *payload, int len)
{
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) return;
    const cJSON *c = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(c)) {
        const char *s = c->valuestring;
        if (strcmp(s, "start") == 0) {
            if (g_cfg.valid) g_start_req = true;
            else dyne_mqtt_pub_status(DYNE_ST_ERR, "\"err\":\"start without cfg\"");
        } else if (strcmp(s, "stop") == 0) {
            g_stop_req = true;
        } else if (strcmp(s, "report") == 0) {
            g_report_req = true;
        } else if (strcmp(s, "reboot") == 0) {
            dyne_mqtt_pub_status(DYNE_ST_ONLINE, NULL); /* retain更新は再起動後 */
            char sub[32];
            snprintf(sub, sizeof(sub), "node/%d/status", dyne_node_id());
            dyne_mqtt_pub(sub, "{\"state\":\"offline\",\"reason\":\"reboot\"}", 1, true, false);
            vTaskDelay(pdMS_TO_TICKS(300));
            esp_restart();
        }
    }
    cJSON_Delete(root);
}

/* ==== WASMアクター ==== */
static wasm_module_inst_t s_inst;
static wasm_exec_env_t s_env;
static wasm_function_inst_t f_init, f_step, f_bval, f_bgrad, f_setn, f_get, f_x;
static wasm_function_inst_t f_setbcv, f_tcoef; /* F1追加の任意エクスポート */

static bool wasm_up(void)
{
    RuntimeInitArgs ia;
    memset(&ia, 0, sizeof(ia));
    ia.mem_alloc_type = Alloc_With_System_Allocator;
    if (!wasm_runtime_full_init(&ia)) return false;
    uint32_t wsize = (uint32_t)(wasm_end - wasm_start);
    uint8_t *wbuf = malloc(wsize);
    if (!wbuf) return false;
    memcpy(wbuf, wasm_start, wsize);
    char err[128];
    wasm_module_t mod = wasm_runtime_load(wbuf, wsize, err, sizeof(err));
    if (!mod) { ESP_LOGE(TAG, "wasm load: %s", err); return false; }
    s_inst = wasm_runtime_instantiate(mod, 16 * 1024, 0, err, sizeof(err));
    if (!s_inst) { ESP_LOGE(TAG, "wasm inst: %s", err); return false; }
    s_env = wasm_runtime_create_exec_env(s_inst, 16 * 1024);
    f_init = wasm_runtime_lookup_function(s_inst, "dyne_init");
    f_step = wasm_runtime_lookup_function(s_inst, "dyne_step");
    f_bval = wasm_runtime_lookup_function(s_inst, "dyne_boundary_value");
    f_bgrad = wasm_runtime_lookup_function(s_inst, "dyne_boundary_grad");
    f_setn = wasm_runtime_lookup_function(s_inst, "dyne_set_neighbor");
    f_get = wasm_runtime_lookup_function(s_inst, "dyne_get");
    f_x = wasm_runtime_lookup_function(s_inst, "dyne_x");
    f_setbcv = wasm_runtime_lookup_function(s_inst, "dyne_set_bc_value");
    f_tcoef = wasm_runtime_lookup_function(s_inst, "dyne_transparent_coef");
    return f_init && f_step && f_bval && f_bgrad && f_setn && f_get && f_x;
}

static double call_d(wasm_function_inst_t f, int i)
{
    wasm_val_t a[1] = { { .kind = WASM_I32, .of.i32 = i } };
    wasm_val_t r[1];
    wasm_runtime_call_wasm_a(s_env, f, 1, r, 1, a);
    return r[0].of.f64;
}

/* F1: 面ごとの設定注入 (dyne-sim の apply_face_config と同じ規則。init の前に呼ぶ) */
static void apply_face_cfg(void)
{
    if (f_setbcv) {
        wasm_val_t a[2];
        a[0].kind = WASM_I32; a[1].kind = WASM_F64;
        a[0].of.i32 = 0; a[1].of.f64 = g_cfg.bc_val_left;
        wasm_runtime_call_wasm_a(s_env, f_setbcv, 0, NULL, 2, a);
        a[0].of.i32 = 1; a[1].of.f64 = g_cfg.bc_val_right;
        wasm_runtime_call_wasm_a(s_env, f_setbcv, 0, NULL, 2, a);
    }
    if (g_cfg.t_m > 0 && f_tcoef) {
        for (int j = 0; j < g_cfg.t_m; j++) {
            wasm_val_t a[4];
            a[0].kind = WASM_I32; a[0].of.i32 = j;
            a[1].kind = WASM_F64; a[1].of.f64 = g_cfg.t_decay[j];
            a[2].kind = WASM_F64; a[2].of.f64 = g_cfg.t_half[j];
            a[3].kind = WASM_F64; a[3].of.f64 = g_cfg.t_weight[j];
            wasm_runtime_call_wasm_a(s_env, f_tcoef, 0, NULL, 4, a);
        }
    }
}

/* ==== 観測の発行 ==== */

static void publish_field(uint64_t step)
{
    /* 担当区間の格子点値 (規約 §2.5)。n≤33 で JSON は out バッファに収まる */
    static char json[3072];
    int off = snprintf(json, sizeof(json),
                       "{\"step\":%llu,\"t\":%.17g,\"x0\":%.17g,\"x1\":%.17g,"
                       "\"n\":%d,\"u\":[",
                       (unsigned long long)step, g_cfg.dt * (double)(step + 1),
                       g_cfg.x0, g_cfg.x1, g_cfg.n);
    for (int i = 0; i < g_cfg.n && off < (int)sizeof(json) - 32; i++) {
        off += snprintf(json + off, sizeof(json) - off, "%s%.17g",
                        i ? "," : "", call_d(f_get, i));
    }
    snprintf(json + off, sizeof(json) - off, "]}");
    char sub[32];
    snprintf(sub, sizeof(sub), "node/%d/field", g_cfg.idx);
    dyne_mqtt_pub(sub, json, 0, false, true);
}

/* リアルタイム率の統計窓 (発注7(v)): ステップ実時間と計算時間、Δt余裕 */
static struct {
    uint32_t n;
    double step_sum, step_min, step_max;
    double comp_sum, comp_min, comp_max;
    double wait_sum;
} s_rt;

static void rt_reset(void)
{
    memset(&s_rt, 0, sizeof(s_rt));
    s_rt.step_min = s_rt.comp_min = 1e30;
}

static void publish_rt(uint64_t step)
{
    if (s_rt.n == 0) return;
    char sub[32], json[384];
    double dt_ms = g_cfg.dt * 1000.0;
    double step_mean = s_rt.step_sum / s_rt.n;
    snprintf(sub, sizeof(sub), "node/%d/rt", g_cfg.idx);
    snprintf(json, sizeof(json),
             "{\"step\":%llu,\"window\":%u,"
             "\"step_ms\":{\"mean\":%.3f,\"min\":%.3f,\"max\":%.3f},"
             "\"compute_ms\":{\"mean\":%.3f,\"min\":%.3f,\"max\":%.3f},"
             "\"wait_ms\":{\"mean\":%.3f},\"dt_ms\":%.6f,\"rt_rate\":%.6g}",
             (unsigned long long)step, (unsigned)s_rt.n,
             step_mean, s_rt.step_min, s_rt.step_max,
             s_rt.comp_sum / s_rt.n, s_rt.comp_min, s_rt.comp_max,
             s_rt.wait_sum / s_rt.n, dt_ms, dt_ms / step_mean);
    dyne_mqtt_pub(sub, json, 0, false, true);
    rt_reset();
}

/* ==== BSP協調 (m2の単一界面ループを最大2界面へ一般化) ==== */

typedef struct {
    int id, face;
    struct sockaddr_in peer;
    struct { uint64_t step; uint8_t buf[DYNE_WIRE_LEN]; bool valid; } slot[2];
    struct { uint64_t step; double v, g; bool valid; } pend;
    bool got;
    double recv_grad;
    int64_t last_send;
} link_t;

static uint64_t stat_sent, stat_resend, stat_replay, stat_crc;

static void link_send_current(int sock, link_t *l, uint64_t step, uint32_t seq)
{
    wasm_val_t a[1] = { { .kind = WASM_I32, .of.i32 = l->face } };
    wasm_val_t r[1];
    wasm_runtime_call_wasm_a(s_env, f_bval, 1, r, 1, a);
    double bv = r[0].of.f64;
    wasm_runtime_call_wasm_a(s_env, f_bgrad, 1, r, 1, a);
    double bg = r[0].of.f64;
    int si = (int)(step & 1);
    dyne_wire_encode(l->slot[si].buf, (uint16_t)l->id, step, seq, bv, bg);
    l->slot[si].step = step;
    l->slot[si].valid = true;
    sendto(sock, l->slot[si].buf, DYNE_WIRE_LEN, 0,
           (struct sockaddr *)&l->peer, sizeof(l->peer));
    stat_sent++;
    l->last_send = esp_timer_get_time();
}

static void link_inject(link_t *l, double v, double g)
{
    wasm_val_t a[3];
    a[0].kind = WASM_I32; a[0].of.i32 = l->face;
    a[1].kind = WASM_F64; a[1].of.f64 = v;
    a[2].kind = WASM_F64; a[2].of.f64 = g;
    wasm_runtime_call_wasm_a(s_env, f_setn, 0, NULL, 3, a);
    l->got = true;
    l->recv_grad = g;
}

static void run_bsp(void)
{
    dyne_cfg_t *c = &g_cfg;
    link_t links[DYNE_MAX_IFACES];
    int nl = c->n_ifaces;
    memset(links, 0, sizeof(links));
    for (int i = 0; i < nl; i++) {
        links[i].id = c->ifaces[i].id;
        links[i].face = c->ifaces[i].face;
        links[i].peer = c->ifaces[i].peer;
    }

    /* アクター初期化 (面設定 → init。dyne-sim と同じ順序) */
    apply_face_cfg();
    {
        wasm_val_t a[7], r[1];
        a[0].kind = WASM_I32; a[0].of.i32 = c->n;
        a[1].kind = WASM_F64; a[1].of.f64 = c->x0;
        a[2].kind = WASM_F64; a[2].of.f64 = c->x1;
        a[3].kind = WASM_F64; a[3].of.f64 = c->alpha;
        a[4].kind = WASM_F64; a[4].of.f64 = c->dt;
        a[5].kind = WASM_I32; a[5].of.i32 = c->bc_left;
        a[6].kind = WASM_I32; a[6].of.i32 = c->bc_right;
        if (!wasm_runtime_call_wasm_a(s_env, f_init, 1, r, 7, a) || r[0].of.i32 != 0) {
            dyne_mqtt_pub_status(DYNE_ST_ERR, "\"err\":\"dyne_init failed\"");
            return;
        }
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in me = { 0 };
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    me.sin_port = htons(CONFIG_DYNE_UDP_PORT);
    bind(sock, (struct sockaddr *)&me, sizeof(me));
    struct timeval tv = { .tv_sec = 0, .tv_usec = RECV_SLICE_MS * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    stat_sent = stat_resend = stat_replay = stat_crc = 0;
    rt_reset();
    uint32_t seq = 0;
    bool aborted = false;
    char sub[32], json[256];

    ESP_LOGI(TAG, "BSP start: %u steps, %d iface(s)", (unsigned)c->steps, nl);
    for (uint64_t step = 0; step < c->steps && !aborted; step++) {
        int64_t t0 = esp_timer_get_time();
        seq++;
        /* 1) 全界面へ送信 (控えを残す) */
        for (int i = 0; i < nl; i++) {
            links[i].got = false;
            link_send_current(sock, &links[i], step, seq);
        }
        /* 2) 取り置き分の即時消費 */
        for (int i = 0; i < nl; i++) {
            if (links[i].pend.valid && links[i].pend.step == step) {
                link_inject(&links[i], links[i].pend.v, links[i].pend.g);
                links[i].pend.valid = false;
            }
        }
        /* 3) 全界面が揃うまで受信・再送 */
        for (;;) {
            bool all = true;
            for (int i = 0; i < nl; i++) all &= links[i].got;
            if (all) break;
            if (g_stop_req) { aborted = true; break; }
            uint8_t rb[DYNE_WIRE_LEN];
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            int rn = recvfrom(sock, rb, sizeof(rb), 0, (struct sockaddr *)&src, &sl);
            int64_t now = esp_timer_get_time();
            if (rn > 0) {
                uint16_t ifid; uint64_t mstep; double mv, mg;
                if (!dyne_wire_decode(rb, rn, &ifid, &mstep, &mv, &mg)) {
                    stat_crc++;
                    continue;
                }
                link_t *l = NULL;
                for (int i = 0; i < nl; i++)
                    if (links[i].id == ifid) l = &links[i];
                if (!l) continue;
                if (mstep == step && !l->got) {
                    link_inject(l, mv, mg);
                } else if (mstep > step) {
                    l->pend.step = mstep; l->pend.v = mv; l->pend.g = mg;
                    l->pend.valid = true;
                } else if (mstep < step) {
                    /* 過去分 = 相手はまだ受け取れていない。控えを再送で応答 */
                    int ri = (int)(mstep & 1);
                    if (l->slot[ri].valid && l->slot[ri].step == mstep) {
                        sendto(sock, l->slot[ri].buf, DYNE_WIRE_LEN, 0,
                               (struct sockaddr *)&l->peer, sizeof(l->peer));
                        stat_replay++;
                    }
                }
            } else {
                /* タイムアウトスライス: バックオフを満たした界面の現ステップを再送 */
                for (int i = 0; i < nl; i++) {
                    if (!links[i].got && now - links[i].last_send >= TIMEOUT_US) {
                        int si = (int)(step & 1);
                        sendto(sock, links[i].slot[si].buf, DYNE_WIRE_LEN, 0,
                               (struct sockaddr *)&links[i].peer, sizeof(links[i].peer));
                        stat_resend++;
                        links[i].last_send = now + RESEND_MIN_US - TIMEOUT_US;
                    }
                }
            }
        }
        if (aborted) break;
        int64_t t1 = esp_timer_get_time();

        /* 界面残差の発行 (規約 §2.9: 界面の左側ノード = 自分の面が右、のみ。
           計算前 = 交換値と同時点の状態で測る。シミュレータの記録規則と同一) */
        if (c->residual_every > 0 && step % (uint64_t)c->residual_every == 0) {
            for (int i = 0; i < nl; i++) {
                if (links[i].face != 1) continue;
                wasm_val_t a[1] = { { .kind = WASM_I32, .of.i32 = 1 } };
                wasm_val_t r[1];
                wasm_runtime_call_wasm_a(s_env, f_bgrad, 1, r, 1, a);
                double resid = fabs(r[0].of.f64 - links[i].recv_grad);
                snprintf(sub, sizeof(sub), "iface/%d/residual", links[i].id);
                snprintf(json, sizeof(json), "{\"step\":%llu,\"residual\":%.6e}",
                         (unsigned long long)step, resid);
                dyne_mqtt_pub(sub, json, 0, false, true);
            }
        }

        /* 4) 計算 */
        wasm_runtime_call_wasm_a(s_env, f_step, 0, NULL, 0, NULL);
        int64_t t2 = esp_timer_get_time();

        /* 統計と観測 */
        double step_ms = (t2 - t0) / 1000.0;
        double comp_ms = (t2 - t1) / 1000.0;
        s_rt.n++;
        s_rt.step_sum += step_ms;
        s_rt.comp_sum += comp_ms;
        s_rt.wait_sum += (t1 - t0) / 1000.0;
        if (step_ms < s_rt.step_min) s_rt.step_min = step_ms;
        if (step_ms > s_rt.step_max) s_rt.step_max = step_ms;
        if (comp_ms < s_rt.comp_min) s_rt.comp_min = comp_ms;
        if (comp_ms > s_rt.comp_max) s_rt.comp_max = comp_ms;

        if (c->field_every > 0 && (step + 1) % (uint64_t)c->field_every == 0)
            publish_field(step);
        if ((c->rt_every > 0 && (step + 1) % (uint64_t)c->rt_every == 0) || g_report_req) {
            publish_rt(step);
            snprintf(json, sizeof(json), "\"step\":%llu", (unsigned long long)(step + 1));
            dyne_mqtt_pub_status(DYNE_ST_RUNNING, json);
            g_report_req = false;
        }
    }

    if (!aborted) {
        /* 終了時ドレイン: 相手の再要求に応答してから締める (M0知見2) */
        int64_t drain_end = esp_timer_get_time() + (int64_t)DRAIN_MS * 1000;
        while (esp_timer_get_time() < drain_end) {
            uint8_t rb[DYNE_WIRE_LEN];
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            int rn = recvfrom(sock, rb, sizeof(rb), 0, (struct sockaddr *)&src, &sl);
            if (rn <= 0) continue;
            uint16_t ifid; uint64_t mstep; double mv, mg;
            if (!dyne_wire_decode(rb, rn, &ifid, &mstep, &mv, &mg)) continue;
            for (int i = 0; i < nl; i++) {
                if (links[i].id != ifid) continue;
                int ri = (int)(mstep & 1);
                if (links[i].slot[ri].valid && links[i].slot[ri].step == mstep) {
                    sendto(sock, links[i].slot[ri].buf, DYNE_WIRE_LEN, 0,
                           (struct sockaddr *)&links[i].peer, sizeof(links[i].peer));
                    stat_replay++;
                }
            }
        }
    }
    close(sock);

    if (aborted) {
        g_stop_req = false;
        dyne_mqtt_pub_status(DYNE_ST_CONFIGURED, "\"note\":\"stopped\"");
        ESP_LOGW(TAG, "BSP stopped by ctl");
        return;
    }

    /* 検証: 温度配列バイト列のCRC32 (M2と同じ) + 解析解誤差 (全域連結時のみ意味を持つ) */
    {
        static uint8_t all[DYNE_MAX_IFACES == 2 ? 8 * 1024 : 8 * 1024]; /* n≤1024点まで */
        double l2 = 0.0, linf = 0.0;
        double decay = exp(-c->alpha * M_PI * M_PI * (c->dt * (double)c->steps));
        for (int i = 0; i < c->n; i++) {
            double u = call_d(f_get, i);
            memcpy(all + i * 8, &u, 8);
            double e = fabs(u - sin(M_PI * call_d(f_x, i)) * decay);
            l2 += e * e;
            if (e > linf) linf = e;
        }
        uint32_t crc = dyne_crc32(all, (size_t)c->n * 8);
        l2 = sqrt(l2 / c->n);
        publish_rt(c->steps - 1);
        publish_field(c->steps - 1);
        char extra[224];
        snprintf(extra, sizeof(extra),
                 "\"crc32\":\"0x%08X\",\"l2\":%.6e,\"linf\":%.6e,"
                 "\"sent\":%llu,\"resend\":%llu,\"replay\":%llu,\"crc_err\":%llu",
                 (unsigned)crc, l2, linf,
                 (unsigned long long)stat_sent, (unsigned long long)stat_resend,
                 (unsigned long long)stat_replay, (unsigned long long)stat_crc);
        dyne_mqtt_pub_status(DYNE_ST_DONE, extra);
        ESP_LOGI(TAG, "BSP done: crc=0x%08X l2=%.3e linf=%.3e", (unsigned)crc, l2, linf);
    }
}

/* ==== メインループ ==== */

static void *node_task(void *arg)
{
    printf("\n==== Dyne node-esp32 (node id %d) ====\n", dyne_node_id());
    wifi_up();
    if (!wasm_up()) {
        ESP_LOGE(TAG, "wasm runtime init failed");
        return NULL;
    }
    ESP_LOGI(TAG, "free heap after wasm: %u", (unsigned)esp_get_free_heap_size());
    dyne_mqtt_start();
    dyne_sensors_start();
    dyne_rssi_start();

    for (;;) {
        if (g_start_req) {
            g_start_req = false;
            if (g_cfg.role == 0) {
                dyne_mqtt_pub_status(DYNE_ST_RUNNING, "\"step\":0");
                run_bsp(); /* 再start可: 再initで場と補助変数はリセットされる */
            } else {
                /* observe役: 計算には参加しない (センサ・RSSIは常時タスクが発行) */
                dyne_mqtt_pub_status(DYNE_ST_RUNNING, "\"note\":\"observe role\"");
            }
        }
        if (g_report_req && !g_cfg.valid) g_report_req = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return NULL;
}

void app_main(void)
{
    pthread_t t;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 32 * 1024);
    if (pthread_create(&t, &attr, node_task, NULL) != 0) {
        printf("[dyne] ERROR: pthread_create\n");
        return;
    }
    pthread_join(t, NULL);
}
