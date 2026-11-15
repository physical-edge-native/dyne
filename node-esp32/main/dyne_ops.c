/*
 * 運用の仕組み (2026-11-11)。宣言は dyne_ops.h、説明は docs/2026-11-11-timesync-ota.md。
 *
 * ctl 命令 (dyne/ctl/<id>。retain の号令は従来どおり無視される):
 *   {"cmd":"ota_fw","job":"J","url":"http://PC:8070/x.bin","size":N,"img_sha256":"hex64","reboot":true}
 *       ファームの書き換え。img_sha256 = .bin の末尾32バイト (ESP-IDF が像に付ける SHA-256)。
 *   {"cmd":"ota_actor","job":"J","name":"heat1d_actor_small",
 *    "wasm":{"url":..,"size":N,"sha256":"hex64"}, "aot":{"url":..,"size":N,"sha256":..}, "reboot":true}
 *       計算部品の書き換え (aot は任意)。いま使っていない方の枠に書き、照合後に見出しを書く。
 *       切り替えは再起動で行う (実行中は受け付けない)。
 *   {"cmd":"actor_rollback","job":"J"}  新しい方の枠の見出しを消して前の計算部品に戻す (再起動する)
 *   {"cmd":"health"}                    health を今すぐ報告
 *
 * 結果 dyne/node/<id>/ota (QoS1、retain なし):
 *   {"job":J,"kind":"fw"|"actor","phase":P, ...}
 *   P = accepted / downloaded (この後に再起動) / ok / rolled_back / fallback / rejected / fail / busy /
 *       selftest_fail (確認待ちの新しいファームが自己確認に失敗し、元に戻す直前)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_partition.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mbedtls/sha256.h"
#include "sdkconfig.h"

#include "dyne_node.h"
#include "dyne_ops.h"
#include "dyne_timesync.h"

static const char *TAG = "dyne-ops";

#define NVS_NS "dyne"
#define ACT_PART_SUBTYPE 0x40
#define ACT_MAX_TRIES 2   /* 新しい計算部品で起動を試みる回数の上限 (それを超えたら飛ばす) */

/* 壊れたファームの試験用の目印 (scripts/dyne_ota.py make-broken が "pass" を "FAIL" に書き換え、
 * SHA-256 を付け直した像を作る。像としては正しいので書き換えは成功し、起動後の自己確認で失敗して
 * 元に戻ることを確かめられる)。この文字列はファームの中に1回だけ現れること */
__attribute__((used)) static const char s_selftest_mark[] = "DYNE_SELFTEST_MARK:pass";

static esp_reset_reason_t s_reset;
static uint32_t s_boot_n, s_prev_up_s;
static bool s_selftest_ok;
static volatile bool s_ota_running;
static struct {
    bool from_slot;
    dyne_actor_cand_t c;
} s_loaded;

/* ==== 小道具 ==== */

static const char *reset_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "pwr_glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu_lockup";
    default: return "unknown";
    }
}

uint32_t dyne_ops_uptime_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

static bool hex32(const char *s, uint8_t out[32])
{
    if (!s || strlen(s) != 64) return false;
    for (int i = 0; i < 32; i++) {
        unsigned v;
        if (sscanf(s + 2 * i, "%2x", &v) != 1) return false;
        out[i] = (uint8_t)v;
    }
    return true;
}

static void nvs_set_str_k(const char *k, const char *v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    if (v) nvs_set_str(h, k, v); else nvs_erase_key(h, k);
    nvs_commit(h);
    nvs_close(h);
}

static bool nvs_get_str_k(const char *k, char *buf, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t l = len;
    bool ok = nvs_get_str(h, k, buf, &l) == ESP_OK;
    nvs_close(h);
    return ok;
}

static void nvs_set_u32_k(const char *k, uint32_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, k, v);
    nvs_commit(h);
    nvs_close(h);
}

static uint32_t nvs_get_u32_k(const char *k, uint32_t def)
{
    nvs_handle_t h;
    uint32_t v = def;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return def;
    if (nvs_get_u32(h, k, &v) != ESP_OK) v = def;
    nvs_close(h);
    return v;
}

static void pub_ota(const char *job, const char *kind, const char *phase, const char *extra)
{
    char sub[32], json[384];
    snprintf(sub, sizeof(sub), "node/%d/ota", dyne_node_id());
    snprintf(json, sizeof(json), "{\"job\":\"%s\",\"kind\":\"%s\",\"phase\":\"%s\",\"up_s\":%u%s%s}",
             job ? job : "", kind, phase, (unsigned)dyne_ops_uptime_s(), extra ? "," : "", extra ? extra : "");
    ESP_LOGI(TAG, "ota: %s", json);
    dyne_mqtt_pub(sub, json, 1, false, false);
}

static void restart_for(const char *reason)
{
    char sub[32], json[96];
    snprintf(sub, sizeof(sub), "node/%d/status", dyne_node_id());
    snprintf(json, sizeof(json), "{\"state\":\"offline\",\"reason\":\"%s\"}", reason);
    dyne_mqtt_pub(sub, json, 1, true, false);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

static bool ota_pending_verify(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    return esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY;
}

static const char *ota_state_name(void)
{
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) != ESP_OK) return "none";
    switch (st) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    default: return "undefined";
    }
}

/* ==== 起動直後 ==== */

void dyne_ops_boot_early(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* 区画表を変えた直後などで NVS の区画の中身が合わない。消して作り直す
         * (NVS に置くのは Wi-Fi の較正値と本ファイルの記録だけで、失っても困らない) */
        ESP_LOGW(TAG, "nvs: %s -> erase", esp_err_to_name(e));
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (e != ESP_OK) ESP_LOGE(TAG, "nvs init: %s", esp_err_to_name(e));
    s_reset = esp_reset_reason();
    s_boot_n = nvs_get_u32_k("boot_n", 0) + 1;
    nvs_set_u32_k("boot_n", s_boot_n);
    s_prev_up_s = nvs_get_u32_k("up_s", 0);
    nvs_set_u32_k("up_s", 0);
    const esp_partition_t *run = esp_ota_get_running_partition();
    printf("[dyne] boot #%u reset=%s prev_uptime=%us part=%s ota_state=%s\n", (unsigned)s_boot_n,
           reset_name(s_reset), (unsigned)s_prev_up_s, run ? run->label : "?", ota_state_name());
}

/* ==== 計算部品の保存区画 ==== */

static const esp_partition_t *act_part(uint32_t *slot_size)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ACT_PART_SUBTYPE, "actors");
    if (p && slot_size) *slot_size = (p->size / 2) / DYNE_ACT_SECTOR * DYNE_ACT_SECTOR;
    return p;
}

/* 両方の枠の見出しを読む */
static void act_read_headers(const esp_partition_t *p, uint32_t ss, dyne_act_hdr_t h[2], bool valid[2],
                             uint32_t seq[2])
{
    for (int i = 0; i < 2; i++) {
        uint8_t b[DYNE_ACT_HDR_LEN];
        valid[i] = false;
        seq[i] = 0;
        if (esp_partition_read(p, (size_t)i * ss, b, sizeof(b)) == ESP_OK
            && dyne_act_hdr_decode(b, ss, &h[i]) == 0) {
            valid[i] = true;
            seq[i] = h[i].seq;
        }
    }
}

int dyne_actor_candidates(dyne_actor_cand_t *out, int max)
{
    uint32_t ss;
    const esp_partition_t *p = act_part(&ss);
    if (!p) return 0;
    dyne_act_hdr_t h[2];
    bool valid[2];
    uint32_t seq[2];
    act_read_headers(p, ss, h, valid, seq);
    int order[2];
    int n = dyne_act_order(valid, seq, order);
    if (n == 0) return 0;
    /* 起動の失敗が続く新しい計算部品を飛ばす */
    uint32_t newest = seq[order[0]];
    uint32_t ok_seq = nvs_get_u32_k("act_ok", 0);
    uint32_t tries = 0;
    if (newest != ok_seq) {
        if (nvs_get_u32_k("act_tseq", 0) != newest) nvs_set_u32_k("act_tseq", newest);
        else tries = nvs_get_u32_k("act_try", 0);
        tries++;
        nvs_set_u32_k("act_try", tries);
    }
    int k = 0;
    for (int i = 0; i < n; i++) {
        int s = order[i];
        if (i == 0 && dyne_act_should_skip(newest, ok_seq, tries, ACT_MAX_TRIES)) {
            printf("[dyne] actor slot%d seq=%u skipped (failed %u boots)\n", s, (unsigned)seq[s], (unsigned)tries - 1);
            continue;
        }
#if CONFIG_WAMR_ENABLE_AOT
        if (h[s].aot_size && k < max) {
            out[k] = (dyne_actor_cand_t){ .slot = s, .aot = true, .seq = seq[s] };
            memcpy(out[k].name, h[s].name, sizeof(out[k].name));
            k++;
        }
#endif
        if (h[s].wasm_size && k < max) {
            out[k] = (dyne_actor_cand_t){ .slot = s, .aot = false, .seq = seq[s] };
            memcpy(out[k].name, h[s].name, sizeof(out[k].name));
            k++;
        }
    }
    return k;
}

uint8_t *dyne_actor_read(const dyne_actor_cand_t *c, uint32_t *len)
{
    uint32_t ss;
    const esp_partition_t *p = act_part(&ss);
    if (!p) return NULL;
    dyne_act_hdr_t h[2];
    bool valid[2];
    uint32_t seq[2];
    act_read_headers(p, ss, h, valid, seq);
    if (!valid[c->slot] || seq[c->slot] != c->seq) return NULL;
    const dyne_act_hdr_t *hh = &h[c->slot];
    uint32_t size = c->aot ? hh->aot_size : hh->wasm_size;
    uint32_t off = (uint32_t)c->slot * ss + (c->aot ? dyne_act_aot_off(hh) : dyne_act_wasm_off(hh));
    const uint8_t *want = c->aot ? hh->aot_sha : hh->wasm_sha;
    uint8_t *buf = dyne_malloc_int_first(size); /* 2026-11-13: 内部RAMを先に (main.c) */
    if (!buf) {
        printf("[dyne] actor slot%d %s: malloc %u failed\n", c->slot, c->aot ? "aot" : "wasm", (unsigned)size);
        return NULL;
    }
    uint8_t got[32];
    if (esp_partition_read(p, off, buf, size) != ESP_OK || mbedtls_sha256(buf, size, got, 0) != 0
        || memcmp(got, want, 32) != 0) {
        printf("[dyne] actor slot%d %s: read/sha256 mismatch\n", c->slot, c->aot ? "aot" : "wasm");
        free(buf);
        return NULL;
    }
    *len = size;
    return buf;
}

void dyne_actor_set_loaded(const dyne_actor_cand_t *c)
{
    s_loaded.from_slot = c != NULL;
    if (c) s_loaded.c = *c;
    printf("[dyne] actor in use: %s\n", c ? (c->aot ? "slot aot" : "slot wasm") : "embedded");
}

static void actor_src_json(char *buf, int len)
{
    if (s_loaded.from_slot)
        snprintf(buf, len, "{\"src\":\"slot%d.%s\",\"seq\":%u,\"name\":\"%s\"}", s_loaded.c.slot,
                 s_loaded.c.aot ? "aot" : "wasm", (unsigned)s_loaded.c.seq, s_loaded.c.name);
    else
        snprintf(buf, len, "{\"src\":\"embedded\",\"seq\":0,\"name\":\"heat1d_actor_small\"}");
}

/* ==== 起動後の自己確認 ==== */

static bool selftest_mark_ok(void)
{
    const volatile char *m = s_selftest_mark;
    return m[19] == 'p' && m[20] == 'a' && m[21] == 's' && m[22] == 's';
}

static void report_jobs(void)
{
    char job[48], tgt[20], extra[200], act[128];
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (nvs_get_str_k("fw_job", job, sizeof(job))) {
        bool hit = nvs_get_str_k("fw_tgt", tgt, sizeof(tgt)) && run && strcmp(run->label, tgt) == 0;
        char elf[17] = "";
        esp_app_get_elf_sha256(elf, sizeof(elf));
        snprintf(extra, sizeof(extra), "\"part\":\"%s\",\"target_part\":\"%s\",\"ver\":\"%s\",\"elf\":\"%s\",\"boot\":%u",
                 run ? run->label : "?", tgt, esp_app_get_description()->version, elf, (unsigned)s_boot_n);
        pub_ota(job, "fw", hit ? "ok" : "rolled_back", extra);
        nvs_set_str_k("fw_job", NULL);
        nvs_set_str_k("fw_tgt", NULL);
    }
    if (nvs_get_str_k("act_job", job, sizeof(job))) {
        uint32_t want = nvs_get_u32_k("act_seq", 0);
        bool hit = s_loaded.from_slot && s_loaded.c.seq == want;
        actor_src_json(act, sizeof(act));
        snprintf(extra, sizeof(extra), "\"want_seq\":%u,\"actor\":%s", (unsigned)want, act);
        pub_ota(job, "actor", hit ? "ok" : "fallback", extra);
        nvs_set_str_k("act_job", NULL);
    }
}

static void selfcheck_task(void *arg)
{
    bool pending = ota_pending_verify();
    int64_t deadline = esp_timer_get_time() + (int64_t)CONFIG_DYNE_OTA_VALIDATE_TIMEOUT_S * 1000000;
    if (pending) printf("[dyne] new firmware pending verify (timeout %ds)\n", CONFIG_DYNE_OTA_VALIDATE_TIMEOUT_S);
    while (!dyne_mqtt_connected() && (!pending || esp_timer_get_time() < deadline))
        vTaskDelay(pdMS_TO_TICKS(200));
    bool mqtt = dyne_mqtt_connected();
    bool mark = selftest_mark_ok();
    bool ok = mqtt && s_selftest_ok && mark;
    if (pending) {
        if (ok) {
            esp_ota_mark_app_valid_cancel_rollback();
            printf("[dyne] firmware marked valid (%.1f s after boot)\n", esp_timer_get_time() / 1e6);
        } else {
            char extra[96];
            snprintf(extra, sizeof(extra), "\"mqtt\":%s,\"selftest\":%s,\"mark\":%s", mqtt ? "true" : "false",
                     s_selftest_ok ? "true" : "false", mark ? "true" : "false");
            printf("[dyne] self-check failed (%s) -> rollback\n", extra);
            if (mqtt) {
                char job[48] = "";
                nvs_get_str_k("fw_job", job, sizeof(job));
                pub_ota(job, "fw", "selftest_fail", extra);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            esp_ota_mark_app_invalid_rollback_and_reboot();
            /* 戻せる先が無い場合だけここに来る */
            ESP_LOGE(TAG, "rollback not possible");
        }
    }
    /* 計算部品: 自己確認に合格したら、使っている通し番号を「合格」として記録 */
    if (ok && s_loaded.from_slot) {
        nvs_set_u32_k("act_ok", s_loaded.c.seq);
        nvs_set_u32_k("act_try", 0);
    }
    while (!dyne_mqtt_connected()) vTaskDelay(pdMS_TO_TICKS(500));
    report_jobs();
    vTaskDelete(NULL);
}

void dyne_ops_start_selfcheck(bool selftest_ok)
{
    s_selftest_ok = selftest_ok;
    xTaskCreate(selfcheck_task, "dyne_selfchk", 4096, NULL, 5, NULL);
}

/* ==== 稼働時間などの定期報告 ==== */

static void publish_health(void)
{
    char sub[32], json[512], ts[200];
    wifi_ap_record_t ap;
    int rssi = esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
    dyne_timesync_summary(ts, sizeof(ts));
    snprintf(sub, sizeof(sub), "node/%d/health", dyne_node_id());
    snprintf(json, sizeof(json),
             "{\"up_s\":%u,\"reset\":\"%s\",\"boot\":%u,\"prev_up_s\":%u,\"heap\":%u,\"heap_min\":%u,"
             "\"largest\":%u,\"ap_rssi\":%d,\"busy\":%s,\"ts_us\":%lld,%s}",
             (unsigned)dyne_ops_uptime_s(), reset_name(s_reset), (unsigned)s_boot_n, (unsigned)s_prev_up_s,
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), rssi,
             g_busy ? "true" : "false", (long long)dyne_time_now_us(), ts);
    dyne_mqtt_pub(sub, json, 0, false, true);
}

static void health_task(void *arg)
{
    uint32_t last_save = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_DYNE_HEALTH_PERIOD_S * 1000));
        if (dyne_mqtt_connected()) publish_health();
        /* 前回の稼働時間 (乾電池が尽きて電源が落ちたときに、どれだけ動いたかを次の起動で報告する)。
         * 60 秒ごとに NVS へ書く (10時間で 600 回。NVS は書き込みを分散するので寿命の問題はない) */
        uint32_t up = dyne_ops_uptime_s();
        if (up - last_save >= 60) {
            nvs_set_u32_k("up_s", up);
            last_save = up;
        }
    }
}

void dyne_ops_start_health(void)
{
    xTaskCreate(health_task, "dyne_health", 4096, NULL, 3, NULL);
}

int dyne_ops_online_fields(char *buf, int len)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    char elf[17] = "", act[128];
    esp_app_get_elf_sha256(elf, sizeof(elf));
    actor_src_json(act, sizeof(act));
    return snprintf(buf, len,
                    "\"target\":\"%s\",\"ver\":\"%s\",\"elf\":\"%s\",\"part\":\"%s\",\"ota_state\":\"%s\","
                    "\"reset\":\"%s\",\"boot\":%u,\"prev_up_s\":%u,\"actor\":%s",
                    CONFIG_IDF_TARGET, esp_app_get_description()->version, elf, run ? run->label : "?",
                    ota_state_name(), reset_name(s_reset), (unsigned)s_boot_n, (unsigned)s_prev_up_s, act);
}

/* ==== ファームの書き換え ==== */

typedef struct {
    char job[40];
    char url[192];
    uint32_t size;
    uint8_t sha[32];
    bool has_sha, reboot;
} fw_req_t;

static void fw_ota_task(void *arg)
{
    fw_req_t *r = arg;
    char extra[200];
    int64_t t0 = esp_timer_get_time();
    const char *phase = "fail", *err = NULL;
    int len = 0;
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *upd = esp_ota_get_next_update_partition(NULL);
    pub_ota(r->job, "fw", "accepted", NULL);
    esp_https_ota_handle_t h = NULL;
    do {
        if (!upd) { err = "no ota partition (old partition table?)"; break; }
        esp_http_client_config_t hc = {
            .url = r->url,
            .timeout_ms = 15000,
            .keep_alive_enable = true,
            .buffer_size = 2048,
        };
        esp_https_ota_config_t oc = { .http_config = &hc };
        esp_err_t e = esp_https_ota_begin(&oc, &h);
        if (e != ESP_OK) { err = "begin (http/connect/image header)"; h = NULL; break; }
        while ((e = esp_https_ota_perform(h)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS) { }
        len = esp_https_ota_get_image_len_read(h);
        if (e != ESP_OK || !esp_https_ota_is_complete_data_received(h)) {
            err = "download";
            esp_https_ota_abort(h);
            break;
        }
        if (r->size && (uint32_t)len != r->size) {
            err = "size mismatch";
            esp_https_ota_abort(h);
            break;
        }
        e = esp_https_ota_finish(h); /* 像の検査 (付属の SHA-256 ほか) と、次に起動する区画の設定 */
        if (e != ESP_OK) {
            phase = e == ESP_ERR_OTA_VALIDATE_FAILED ? "rejected" : "fail";
            err = e == ESP_ERR_OTA_VALIDATE_FAILED ? "image invalid (checksum/sha256/chip)" : "finish";
            break;
        }
        if (r->has_sha) {
            uint8_t got[32];
            if (esp_partition_get_sha256(upd, got) != ESP_OK || memcmp(got, r->sha, 32) != 0) {
                esp_ota_set_boot_partition(run); /* 起動先を元に戻す */
                phase = "rejected";
                err = "img_sha256 mismatch";
                break;
            }
        }
        phase = "downloaded";
    } while (0);
    int ms = (int)((esp_timer_get_time() - t0) / 1000);
    snprintf(extra, sizeof(extra), "\"ms\":%d,\"bytes\":%d,\"from_part\":\"%s\",\"to_part\":\"%s\"%s%s%s", ms, len,
             run ? run->label : "?", upd ? upd->label : "?", err ? ",\"err\":\"" : "", err ? err : "", err ? "\"" : "");
    if (!err) { /* phase = "downloaded" */
        nvs_set_str_k("fw_job", r->job);
        nvs_set_str_k("fw_tgt", upd->label);
        pub_ota(r->job, "fw", phase, extra);
        if (r->reboot && !g_busy) restart_for("ota_fw");
    } else {
        pub_ota(r->job, "fw", phase, extra);
    }
    free(r);
    s_ota_running = false;
    vTaskDelete(NULL);
}

/* ==== 計算部品の書き換え ==== */

typedef struct {
    char url[192];
    uint32_t size;
    uint8_t sha[32];
} blob_req_t;

typedef struct {
    char job[40];
    char name[DYNE_ACT_NAME_LEN];
    blob_req_t wasm, aot;
    bool has_wasm, has_aot, reboot;
} act_req_t;

/* HTTP で取ってきて区画へ書く。SHA-256 と長さを照合する。戻り値: NULL = 成功 / 誤りの説明。
 * *rejected = 照合の不一致 (届いた中身が目録と違う) */
static const char *http_to_part(const esp_partition_t *p, uint32_t off, const blob_req_t *b, bool *rejected,
                                uint32_t *got_len)
{
    const char *err = NULL;
    *rejected = false;
    *got_len = 0;
    esp_http_client_config_t hc = { .url = b->url, .timeout_ms = 15000 };
    esp_http_client_handle_t cl = esp_http_client_init(&hc);
    if (!cl) return "http init";
    uint8_t *buf = malloc(2048);
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    do {
        if (!buf) { err = "malloc"; break; }
        if (esp_http_client_open(cl, 0) != ESP_OK) { err = "http open"; break; }
        esp_http_client_fetch_headers(cl);
        if (esp_http_client_get_status_code(cl) != 200) { err = "http status"; break; }
        uint32_t total = 0;
        int n;
        while ((n = esp_http_client_read(cl, (char *)buf, 2048)) > 0) {
            if (total + (uint32_t)n > b->size) { err = "longer than manifest size"; *rejected = true; break; }
            if (esp_partition_write(p, off + total, buf, n) != ESP_OK) { err = "flash write"; break; }
            mbedtls_sha256_update(&ctx, buf, n);
            total += (uint32_t)n;
        }
        *got_len = total;
        if (err) break;
        if (n < 0) { err = "http read"; break; }
        if (total != b->size) { err = "size mismatch"; *rejected = true; break; }
        uint8_t got[32];
        mbedtls_sha256_finish(&ctx, got);
        if (memcmp(got, b->sha, 32) != 0) { err = "sha256 mismatch"; *rejected = true; break; }
    } while (0);
    mbedtls_sha256_free(&ctx);
    free(buf);
    esp_http_client_close(cl);
    esp_http_client_cleanup(cl);
    return err;
}

static void act_ota_task(void *arg)
{
    act_req_t *r = arg;
    char extra[200];
    int64_t t0 = esp_timer_get_time();
    const char *phase = "fail", *err = NULL;
    uint32_t bytes = 0, seq_new = 0;
    int w = -1;
    bool erased = false;
    pub_ota(r->job, "actor", "accepted", NULL);
    uint32_t ss;
    const esp_partition_t *p = act_part(&ss);
    do {
        if (!p) { err = "no actors partition (old partition table?)"; break; }
        dyne_act_hdr_t h[2];
        bool valid[2];
        uint32_t seq[2];
        act_read_headers(p, ss, h, valid, seq);
        /* いま使っている計算部品の枠は残す (使っていなければ、新しくない方の枠) */
        w = s_loaded.from_slot ? 1 - s_loaded.c.slot : dyne_act_write_slot(valid, seq);
        seq_new = dyne_act_next_seq(valid, seq);
        dyne_act_hdr_t nh = { .seq = seq_new };
        if (r->has_wasm) { nh.wasm_size = r->wasm.size; memcpy(nh.wasm_sha, r->wasm.sha, 32); }
        if (r->has_aot) { nh.aot_size = r->aot.size; memcpy(nh.aot_sha, r->aot.sha, 32); }
        strlcpy(nh.name, r->name, sizeof(nh.name));
        uint32_t span = dyne_act_span(nh.wasm_size, nh.aot_size);
        if (span > ss) { err = "too large for slot"; phase = "rejected"; break; }
        uint32_t base = (uint32_t)w * ss;
        /* 見出しの消去 (この時点で枠 w は「空」) → 本体 → 照合 → 見出し */
        erased = true;
        if (esp_partition_erase_range(p, base, span) != ESP_OK) { err = "erase"; break; }
        bool rej;
        uint32_t got;
        if (r->has_wasm) {
            err = http_to_part(p, base + dyne_act_wasm_off(&nh), &r->wasm, &rej, &got);
            bytes += got;
            if (err) { if (rej) phase = "rejected"; break; }
        }
        if (r->has_aot) {
            err = http_to_part(p, base + dyne_act_aot_off(&nh), &r->aot, &rej, &got);
            bytes += got;
            if (err) { if (rej) phase = "rejected"; break; }
        }
        uint8_t hb[DYNE_ACT_HDR_LEN];
        dyne_act_hdr_encode(hb, &nh);
        if (esp_partition_write(p, base, hb, sizeof(hb)) != ESP_OK) { err = "header write"; break; }
        phase = "downloaded";
    } while (0);
    if (err && erased) esp_partition_erase_range(p, (uint32_t)w * ss, DYNE_ACT_SECTOR); /* 枠を空に */
    int ms = (int)((esp_timer_get_time() - t0) / 1000);
    snprintf(extra, sizeof(extra), "\"ms\":%d,\"bytes\":%u,\"slot\":%d,\"seq\":%u%s%s%s", ms, (unsigned)bytes, w,
             (unsigned)seq_new, err ? ",\"err\":\"" : "", err ? err : "", err ? "\"" : "");
    if (!err) { /* phase = "downloaded" */
        nvs_set_str_k("act_job", r->job);
        nvs_set_u32_k("act_seq", seq_new);
        nvs_set_u32_k("act_tseq", seq_new);
        nvs_set_u32_k("act_try", 0);
        pub_ota(r->job, "actor", phase, extra);
        if (r->reboot && !g_busy) restart_for("ota_actor");
    } else {
        pub_ota(r->job, "actor", phase, extra);
    }
    free(r);
    s_ota_running = false;
    vTaskDelete(NULL);
}

static bool parse_blob(const cJSON *o, blob_req_t *b)
{
    if (!cJSON_IsObject(o)) return false;
    const cJSON *u = cJSON_GetObjectItem(o, "url"), *s = cJSON_GetObjectItem(o, "size"),
                *h = cJSON_GetObjectItem(o, "sha256");
    if (!cJSON_IsString(u) || !cJSON_IsNumber(s) || !cJSON_IsString(h)) return false;
    strlcpy(b->url, u->valuestring, sizeof(b->url));
    b->size = (uint32_t)s->valuedouble;
    return b->size > 0 && hex32(h->valuestring, b->sha);
}

static const char *str_or(const cJSON *root, const char *k, const char *def)
{
    const cJSON *v = cJSON_GetObjectItem(root, k);
    return cJSON_IsString(v) ? v->valuestring : def;
}

static bool bool_or(const cJSON *root, const char *k, bool def)
{
    const cJSON *v = cJSON_GetObjectItem(root, k);
    return cJSON_IsBool(v) ? cJSON_IsTrue(v) : def;
}

bool dyne_ops_on_ctl(const char *cmd, const cJSON *root)
{
    if (strcmp(cmd, "timesync") == 0) {
        dyne_timesync_on_ctl(root);
        return true;
    }
    if (strcmp(cmd, "health") == 0) {
        publish_health();
        return true;
    }
    bool fw = strcmp(cmd, "ota_fw") == 0, act = strcmp(cmd, "ota_actor") == 0;
    bool rb = strcmp(cmd, "actor_rollback") == 0;
    if (!fw && !act && !rb) return false;
    const char *job = str_or(root, "job", "");
    const char *kind = fw ? "fw" : "actor";
    /* 実行中 (計算の最中) と、別の書き換えの最中は受け付けない */
    if (g_busy || s_ota_running) {
        pub_ota(job, kind, "busy", g_busy ? "\"err\":\"running\"" : "\"err\":\"ota in progress\"");
        return true;
    }
    if (rb) {
        uint32_t ss;
        const esp_partition_t *p = act_part(&ss);
        dyne_act_hdr_t h[2];
        bool valid[2];
        uint32_t seq[2];
        int order[2];
        if (!p) { pub_ota(job, kind, "fail", "\"err\":\"no actors partition\""); return true; }
        act_read_headers(p, ss, h, valid, seq);
        if (dyne_act_order(valid, seq, order) == 0) { pub_ota(job, kind, "fail", "\"err\":\"no slot\""); return true; }
        esp_partition_erase_range(p, (uint32_t)order[0] * ss, DYNE_ACT_SECTOR);
        char extra[64];
        snprintf(extra, sizeof(extra), "\"erased_slot\":%d,\"erased_seq\":%u", order[0], (unsigned)seq[order[0]]);
        nvs_set_str_k("act_job", job);
        nvs_set_u32_k("act_seq", valid[1 - order[0]] ? seq[1 - order[0]] : 0);
        pub_ota(job, kind, "downloaded", extra);
        restart_for("actor_rollback");
        return true;
    }
    if (fw) {
        fw_req_t *r = calloc(1, sizeof(*r));
        const cJSON *u = cJSON_GetObjectItem(root, "url"), *s = cJSON_GetObjectItem(root, "size");
        if (!r || !cJSON_IsString(u)) { free(r); pub_ota(job, kind, "fail", "\"err\":\"bad command\""); return true; }
        strlcpy(r->job, job, sizeof(r->job));
        strlcpy(r->url, u->valuestring, sizeof(r->url));
        r->size = cJSON_IsNumber(s) ? (uint32_t)s->valuedouble : 0;
        r->has_sha = hex32(str_or(root, "img_sha256", NULL), r->sha);
        r->reboot = bool_or(root, "reboot", true);
        s_ota_running = true;
        if (xTaskCreate(fw_ota_task, "dyne_otafw", 8192, r, 5, NULL) != pdPASS) {
            s_ota_running = false;
            free(r);
            pub_ota(job, kind, "fail", "\"err\":\"task\"");
        }
        return true;
    }
    act_req_t *r = calloc(1, sizeof(*r));
    if (!r) { pub_ota(job, kind, "fail", "\"err\":\"malloc\""); return true; }
    strlcpy(r->job, job, sizeof(r->job));
    strlcpy(r->name, str_or(root, "name", "actor"), sizeof(r->name));
    r->has_wasm = parse_blob(cJSON_GetObjectItem(root, "wasm"), &r->wasm);
    r->has_aot = parse_blob(cJSON_GetObjectItem(root, "aot"), &r->aot);
    r->reboot = bool_or(root, "reboot", true);
    if (!r->has_wasm && !r->has_aot) {
        free(r);
        pub_ota(job, kind, "fail", "\"err\":\"bad command (wasm/aot url,size,sha256)\"");
        return true;
    }
    s_ota_running = true;
    if (xTaskCreate(act_ota_task, "dyne_otaact", 6144, r, 5, NULL) != pdPASS) {
        s_ota_running = false;
        free(r);
        pub_ota(job, kind, "fail", "\"err\":\"task\"");
    }
    return true;
}
