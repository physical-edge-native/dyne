/*
 * IMU (Arduino Modulino Movement = LSM6DSOX、I2C) (2026-11-13、2週間計画の G12・実験9)
 *
 * 説明: docs/2026-11-13-firmware-nodes.md。FIFO の語の解釈と時刻の当てはめは dyne_sense_core.c
 * (PC で検査済み)。
 *
 * 動き:
 *  - 起動時 (menuconfig の DYNE_IMU_AUTO が y のとき) に番地 0x6A・0x6B を探し、WHO_AM_I = 0x6C の
 *    ものを使う。
 *  - センサ内の FIFO (ため置き) に、加速度・角速度と、センサの時刻印 (25µs 刻みの計数) を同じ周期で
 *    ためさせる (時刻印は各標本の前に1語)。batch_ms ごとに FIFO を読み出す。
 *  - センサの時刻印 → esp_timer: 読み出しのたびに時刻印のレジスタを読み、その前後の esp_timer の
 *    中点と組にして直近 64 組に直線を当てはめる (センサの発振の誤差 ±1.5% 程度を吸収する)。
 *    esp_timer → 同期した時刻 は時刻同期のモジュール (dyne_timesync.c) の推定を使う。
 *  - 生の標本を dyne/node/<id>/imu に発行する (実験9の記録用)。同時に、ワイヤ形式v2 の経路が
 *    IMU の融合のアクター (crates/imu-fusion-actor) の dyne_imu_push へ渡すため、ため置き (256標本) に入れる。
 *
 * 発行の形 (1回の読み出しぶん。標本が多ければ複数に分ける):
 *   {"seq":N,"odr":104,"fs_g":4,"fs_dps":500,"sa":<g/LSB>,"sg":<dps/LSB>,"synced":true,
 *    "us_per_tick":25.03,"fit_resid_us":40,"ovr":0,"orphan":0,
 *    "s":[[t_us, t_loc, ax, ay, az, gx, gy, gz], ...]}
 *   t_us = 同期した時刻 [µs] (未同期は 0)、t_loc = esp_timer [µs]、ax..gz = 生の値 (符号つき16ビット)。
 *   物理量 = 生の値 × sa [g] / × sg [dps]
 *
 * ctl 命令: {"cmd":"imu", "en":true, "probe":false, "odr":104, "fs_g":4, "fs_dps":500,
 *            "batch_ms":100, "pub":true}
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "dyne_node.h"
#include "dyne_sense.h"
#include "dyne_timesync.h"

static const char *TAG = "dyne-imu";

/* LSM6DSOX のレジスタ (資料 DS12814) */
#define R_FIFO_CTRL3   0x09
#define R_FIFO_CTRL4   0x0A
#define R_WHO_AM_I     0x0F
#define R_CTRL1_XL     0x10
#define R_CTRL2_G      0x11
#define R_CTRL3_C      0x12
#define R_CTRL9_XL     0x18
#define R_CTRL10_C     0x19
#define R_FIFO_STATUS1 0x3A
#define R_TIMESTAMP0   0x40
#define R_FREQ_FINE    0x63
#define R_FIFO_OUT_TAG 0x78
#define WHO_AM_I_VAL   0x6C

#define RING_N 256
#define G0 9.80665

typedef struct {
    bool en, pub;
    double odr;
    int fs_g, fs_dps, batch_ms;
} imu_cfg_t;

static imu_cfg_t s_cfg, s_pend;
static volatile bool s_apply, s_probe;
static SemaphoreHandle_t s_mx;

static i2c_master_dev_handle_t s_dev;
static int s_addr = -1;
static bool s_found, s_running, s_task_up;
static char s_err[48];
static dyne_imu_parse_t s_parse;
static dyne_imu_clock_t *s_clkp; /* IMU が見つかったときにだけ確保 (静的 RAM を増やさない) */
#define s_clk (*s_clkp)
static uint32_t s_batch_seq;
static uint64_t s_ovr, s_i2c_err;
static double s_sa, s_sg; /* g/LSB, dps/LSB */

/* アクターへ渡す標本のため置き (生の値で持ち、取り出すときに SI 単位へ) */
typedef struct { int64_t t; bool synced; int16_t v[6]; } ring_ent_t;
static ring_ent_t *s_ring;
static uint64_t s_wr;              /* 書いた総数 */
static int64_t s_newest_t;
static portMUX_TYPE s_rmux = portMUX_INITIALIZER_UNLOCKED;

/* 発行の組み立て (IMU が見つかったときにだけ確保) */
#define PUB_CAP 3600
static char *s_pub; /* PUB_CAP + 128。IMU が見つかったときにだけ確保 */
#define PUB_SZ (PUB_CAP + 128)

static void default_cfg(imu_cfg_t *c)
{
    c->en = CONFIG_DYNE_IMU_AUTO;
    c->pub = true;
    c->odr = 104;
    c->fs_g = 4;
    c->fs_dps = 500;
    c->batch_ms = 100;
}

static bool wr(uint8_t reg, uint8_t v)
{
    uint8_t b[2] = { reg, v };
    bool ok = i2c_master_transmit(s_dev, b, 2, 50) == ESP_OK;
    if (!ok) s_i2c_err++;
    return ok;
}
static bool rd(uint8_t reg, uint8_t *buf, size_t n)
{
    bool ok = i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 50) == ESP_OK;
    if (!ok) s_i2c_err++;
    return ok;
}

static bool probe_dev(void);
/* 見つかったら作業領域を確保する (見つからない基板の静的 RAM・ヒープを減らさない) */
static bool probe(void)
{
    if (!probe_dev()) return false;
    if (!s_clkp) s_clkp = calloc(1, sizeof(dyne_imu_clock_t));
    if (!s_pub) s_pub = malloc(PUB_SZ);
    if (!s_ring) s_ring = calloc(RING_N, sizeof(ring_ent_t)); /* 約6KB */
    if (!s_clkp || !s_pub || !s_ring) { snprintf(s_err, sizeof(s_err), "alloc"); return false; }
    return true;
}

static bool probe_dev(void)
{
    i2c_master_bus_handle_t bus = dyne_i2c_bus();
    if (!bus) { snprintf(s_err, sizeof(s_err), "no i2c bus"); return false; }
    static const int addrs[2] = { 0x6A, 0x6B };
    for (int i = 0; i < 2; i++) {
        if (i2c_master_probe(bus, addrs[i], 50) != ESP_OK) continue;
        i2c_master_dev_handle_t h;
        i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addrs[i],
                                   .scl_speed_hz = 400000 };
        if (i2c_master_bus_add_device(bus, &dc, &h) != ESP_OK) continue;
        uint8_t reg = R_WHO_AM_I, who = 0;
        if (i2c_master_transmit_receive(h, &reg, 1, &who, 1, 50) == ESP_OK && who == WHO_AM_I_VAL) {
            s_dev = h;
            s_addr = addrs[i];
            s_err[0] = 0;
            ESP_LOGI(TAG, "LSM6DSOX at 0x%02X", s_addr);
            return true;
        }
        i2c_master_bus_rm_device(h);
    }
    snprintf(s_err, sizeof(s_err), "LSM6DSOX not found at 0x6A/0x6B");
    return false;
}

static bool configure(void)
{
    s_running = false;
    int odr = dyne_lsm_odr_code(s_cfg.odr), fx = dyne_lsm_fs_xl_bits(s_cfg.fs_g), fgy = dyne_lsm_fs_g_bits(s_cfg.fs_dps);
    if (odr < 0 || fx < 0 || fgy < 0) { snprintf(s_err, sizeof(s_err), "bad odr/fs"); return false; }
    bool ok = wr(R_CTRL3_C, 0x01); /* SW_RESET */
    vTaskDelay(pdMS_TO_TICKS(20) > 0 ? pdMS_TO_TICKS(20) : 2);
    if (!s_cfg.en) { /* 止める: 電源投入時と同じ (測らない) */
        return ok;
    }
    ok &= wr(R_CTRL3_C, 0x44);            /* BDU (上下のバイトの食い違いを防ぐ) | IF_INC (連続読み) */
    ok &= wr(R_CTRL9_XL, 0xE2);           /* I3C を切る (I2C だけで使う。資料の推奨) */
    ok &= wr(R_FIFO_CTRL4, 0x00);         /* FIFO をいったん止めて空にする */
    ok &= wr(R_CTRL1_XL, (uint8_t)((odr << 4) | (fx << 2)));
    ok &= wr(R_CTRL2_G, (uint8_t)((odr << 4) | (fgy << 2)));
    ok &= wr(R_CTRL10_C, 0x20);           /* TIMESTAMP_EN */
    ok &= wr(R_FIFO_CTRL3, (uint8_t)((odr << 4) | odr)); /* 角速度・加速度を同じ周期でためる */
    ok &= wr(R_FIFO_CTRL4, 0x46);         /* 時刻印を毎回 (DEC_TS_BATCH=01) | 連続モード (110) */
    uint8_t ff = 0;
    ok &= rd(R_FREQ_FINE, &ff, 1);
    if (!ok) { snprintf(s_err, sizeof(s_err), "i2c write"); return false; }
    s_sa = dyne_lsm_acc_sens_g(s_cfg.fs_g);
    s_sg = dyne_lsm_gyro_sens_dps(s_cfg.fs_dps);
    dyne_imu_parse_init(&s_parse);
    dyne_imu_clock_init(&s_clk, dyne_lsm_us_per_tick((int8_t)ff));
    portENTER_CRITICAL(&s_rmux);
    s_wr = 0;
    s_newest_t = 0;
    portEXIT_CRITICAL(&s_rmux);
    s_running = true;
    ESP_LOGI(TAG, "running: odr=%.1f fs=%dg/%ddps freq_fine=%d (%.3f us/tick)", s_cfg.odr, s_cfg.fs_g,
             s_cfg.fs_dps, (int8_t)ff, s_clk.nominal);
    return true;
}

static void publish_state(void)
{
    char sub[32], json[384];
    snprintf(sub, sizeof(sub), "node/%d/sense", dyne_node_id());
    snprintf(json, sizeof(json),
             "{\"imu\":{\"found\":%s,\"addr\":%d,\"running\":%s,\"odr\":%.1f,\"fs_g\":%d,\"fs_dps\":%d,"
             "\"batch_ms\":%d,\"pub\":%s,\"samples\":%llu,\"ovr\":%llu,\"orphan\":%llu,\"i2c_err\":%llu,"
             "\"us_per_tick\":%.4f,\"fit_resid_us\":%.0f,\"err\":\"%s\"}}",
             s_found ? "true" : "false", s_addr, s_running ? "true" : "false", s_cfg.odr, s_cfg.fs_g, s_cfg.fs_dps,
             s_cfg.batch_ms, s_cfg.pub ? "true" : "false", (unsigned long long)s_parse.n_samples,
             (unsigned long long)s_ovr, (unsigned long long)s_parse.n_orphan, (unsigned long long)s_i2c_err,
             s_clkp ? s_clk.rate : 0.0, s_clkp ? s_clk.resid_max : 0.0, s_err);
    dyne_mqtt_pub(sub, json, 0, false, true);
}

static int s_pub_n, s_pub_cnt;

static void pub_begin(void)
{
    s_pub_n = snprintf(s_pub, PUB_SZ,
                       "{\"seq\":%u,\"odr\":%.1f,\"fs_g\":%d,\"fs_dps\":%d,\"sa\":%.6e,\"sg\":%.6e,\"synced\":%s,"
                       "\"us_per_tick\":%.5f,\"fit_resid_us\":%.0f,\"ovr\":%llu,\"orphan\":%llu,\"s\":[",
                       (unsigned)++s_batch_seq, s_cfg.odr, s_cfg.fs_g, s_cfg.fs_dps, s_sa, s_sg,
                       dyne_time_synced() ? "true" : "false", s_clk.rate, s_clk.resid_max,
                       (unsigned long long)s_ovr, (unsigned long long)s_parse.n_orphan);
    s_pub_cnt = 0;
}

static void pub_flush(void)
{
    if (s_pub_cnt == 0) return;
    s_pub_n += snprintf(s_pub + s_pub_n, PUB_SZ - s_pub_n, "]}");
    char sub[32];
    snprintf(sub, sizeof(sub), "node/%d/imu", dyne_node_id());
    dyne_mqtt_pub(sub, s_pub, 0, false, true);
    s_pub_cnt = 0;
}

static void sample_out(const dyne_imu_raw_t *r)
{
    int64_t loc = dyne_imu_clock_local(&s_clk, r->tick);
    int64_t ts = dyne_time_at_local(loc);
    ring_ent_t e = { .t = ts > 0 ? ts : loc, .synced = ts > 0 };
    for (int k = 0; k < 3; k++) { e.v[k] = r->a[k]; e.v[3 + k] = r->g[k]; }
    if (s_ring) {
        portENTER_CRITICAL(&s_rmux);
        s_ring[s_wr % RING_N] = e;
        s_wr++;
        s_newest_t = e.t;
        portEXIT_CRITICAL(&s_rmux);
    }
    if (!s_cfg.pub) return;
    if (s_pub_cnt == 0) pub_begin();
    s_pub_n += snprintf(s_pub + s_pub_n, PUB_SZ - s_pub_n, "%s[%lld,%lld,%d,%d,%d,%d,%d,%d]",
                        s_pub_cnt ? "," : "", (long long)ts, (long long)loc, r->a[0], r->a[1], r->a[2],
                        r->g[0], r->g[1], r->g[2]);
    s_pub_cnt++;
    if (s_pub_n > PUB_CAP) pub_flush();
}

static void drain(void)
{
    /* 時刻印のレジスタと esp_timer の組 (読み出しの前後の中点) */
    uint8_t t4[4];
    int64_t tb = esp_timer_get_time();
    bool ok = rd(R_TIMESTAMP0, t4, 4);
    int64_t ta = esp_timer_get_time();
    if (ok && ta - tb < 2000) {
        uint32_t raw = (uint32_t)t4[0] | ((uint32_t)t4[1] << 8) | ((uint32_t)t4[2] << 16) | ((uint32_t)t4[3] << 24);
        dyne_imu_clock_add(&s_clk, dyne_imu_unwrap(&s_parse.wrap, raw), (tb + ta) / 2);
    }
    uint8_t st[2];
    if (!rd(R_FIFO_STATUS1, st, 2)) return;
    int level = st[0] | ((st[1] & 0x03) << 8);
    if (st[1] & 0x40) s_ovr++; /* FIFO_OVR_IA: 読み出しが遅れて標本を失った */
    for (int i = 0; i < level; i++) {
        uint8_t w[7];
        if (!rd(R_FIFO_OUT_TAG, w, 7)) break;
        dyne_imu_raw_t r;
        if (dyne_imu_fifo_word(&s_parse, w, &r)) sample_out(&r);
    }
    pub_flush();
}

static void imu_task(void *arg)
{
    if (s_cfg.en) s_found = probe();
    if (s_found) configure();
    publish_state();
    int64_t last_state = esp_timer_get_time();
    for (;;) {
        if (s_apply) {
            xSemaphoreTake(s_mx, portMAX_DELAY);
            s_cfg = s_pend;
            s_apply = false;
            xSemaphoreGive(s_mx);
            if (!s_found && (s_probe || s_cfg.en)) s_found = probe();
            s_probe = false;
            if (s_found) configure();
            publish_state();
        }
        if (s_running) drain();
        int64_t now = esp_timer_get_time();
        if (now - last_state > 10000000) {
            publish_state();
            last_state = now;
        }
        TickType_t d = pdMS_TO_TICKS(s_cfg.batch_ms);
        vTaskDelay(d > 0 ? d : 1);
    }
}

void dyne_imu_start(void)
{
    if (!s_mx) s_mx = xSemaphoreCreateMutex();
    default_cfg(&s_cfg);
    s_pend = s_cfg;
    if (!dyne_i2c_bus()) return;
    if (xTaskCreate(imu_task, "dyne_imu", 4096, NULL, 4, NULL) == pdPASS) s_task_up = true;
}

bool dyne_imu_active(void)
{
    return s_running;
}

int64_t dyne_imu_newest_t(void)
{
    portENTER_CRITICAL(&s_rmux);
    int64_t t = s_newest_t;
    portEXIT_CRITICAL(&s_rmux);
    return t;
}

int dyne_imu_take(dyne_imu_sample_t *out, int max, uint64_t *rd_pos, uint64_t *lost)
{
    *lost = 0;
    if (!s_ring) return 0;
    int n = 0;
    double sa = s_sa * G0, sg = s_sg * M_PI / 180.0;
    portENTER_CRITICAL(&s_rmux);
    uint64_t wr_pos = s_wr;
    if (*rd_pos == 0 || *rd_pos > wr_pos) *rd_pos = wr_pos; /* 初回・設定し直しの後は今から */
    if (wr_pos - *rd_pos > RING_N) {
        *lost = wr_pos - *rd_pos - RING_N;
        *rd_pos = wr_pos - RING_N;
    }
    while (*rd_pos < wr_pos && n < max) {
        const ring_ent_t *e = &s_ring[*rd_pos % RING_N];
        out[n].t_sync_us = e->t;
        out[n].t_synced = e->synced;
        for (int k = 0; k < 3; k++) {
            out[n].a[k] = e->v[k] * sa;
            out[n].g[k] = e->v[3 + k] * sg;
        }
        n++;
        (*rd_pos)++;
    }
    portEXIT_CRITICAL(&s_rmux);
    return n;
}

bool dyne_imu_on_ctl(const char *cmd, const cJSON *root)
{
    if (strcmp(cmd, "imu") != 0) return false;
    char sub[40], json[128];
    snprintf(sub, sizeof(sub), "node/%d/sense/ack", dyne_node_id());
    if (!s_task_up) {
        snprintf(json, sizeof(json), "{\"cmd\":\"imu\",\"ok\":false,\"err\":\"imu task not running\"}");
        dyne_mqtt_pub(sub, json, 1, false, false);
        return true;
    }
    xSemaphoreTake(s_mx, portMAX_DELAY);
    imu_cfg_t c = s_apply ? s_pend : s_cfg;
    xSemaphoreGive(s_mx);
    const cJSON *v;
    if ((v = cJSON_GetObjectItem(root, "en")) && cJSON_IsBool(v)) c.en = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(root, "pub")) && cJSON_IsBool(v)) c.pub = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(root, "probe")) && cJSON_IsTrue(v)) s_probe = true;
    if ((v = cJSON_GetObjectItem(root, "odr")) && cJSON_IsNumber(v)) c.odr = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "fs_g")) && cJSON_IsNumber(v)) c.fs_g = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "fs_dps")) && cJSON_IsNumber(v)) c.fs_dps = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "batch_ms")) && cJSON_IsNumber(v)) c.batch_ms = v->valueint;
    const char *err = NULL;
    if (dyne_lsm_odr_code(c.odr) < 0) err = "odr: 12.5/26/52/104/208/416/833";
    else if (dyne_lsm_fs_xl_bits(c.fs_g) < 0) err = "fs_g: 2/4/8/16";
    else if (dyne_lsm_fs_g_bits(c.fs_dps) < 0) err = "fs_dps: 250/500/1000/2000";
    /* FIFO は少なくとも 3KB (= 7バイトの語 約 438 語) と見て、1標本 3語なので、読み出しの間隔に入る
     * 標本は 120 までにする (余裕を見た値。FIFO の実際の大きさは資料で要確認) */
    else if (c.batch_ms < 10 || c.batch_ms * c.odr / 1000.0 > 120) err = "batch_ms too long for odr (FIFO)";
    if (!err) {
        xSemaphoreTake(s_mx, portMAX_DELAY);
        s_pend = c;
        s_apply = true;
        xSemaphoreGive(s_mx);
    }
    if (err) snprintf(json, sizeof(json), "{\"cmd\":\"imu\",\"ok\":false,\"err\":\"%s\"}", err);
    else snprintf(json, sizeof(json), "{\"cmd\":\"imu\",\"ok\":true}");
    dyne_mqtt_pub(sub, json, 1, false, false);
    return true;
}
