/*
 * 測距 (VL53L5CX) とサーボの首振り (PAN4H) (2026-11-13、2週間計画の G3・P11)
 *
 * 説明: docs/2026-11-13-firmware-nodes.md。手順の計算部分は dyne_sense_core.c (PC で検査済み)。
 *
 * ドライバ: ESP-IDF コンポーネント rjrp44/vl53l5cx 4.0.1 (ST の ULD 2.0.0 の移植。BSD-3-Clause の
 * 条件で使う。vl53-eval/ で評価済みのものと同じ版)。I2C のバスは sensors.c と共有 (番地 0x29)。
 *
 * 動き:
 *  - 起動時 (menuconfig の DYNE_RANGE_AUTO が y のとき) に 0x29 を探し、居ればセンサへファーム (約80KB) を
 *    書き込んで (約2.3秒) 測距を始める。居なければ何もしない (ctl の range {"probe":true} で探し直す)。
 *  - フレームごとに、首振りなら dyne_sense_core の手順で「採る (センサID = id_base + 位置) / 捨てる」を決める。
 *    採ったフレームは行の取り出し・左右の反転をして最新の1つを保持し (ワイヤ形式v2 の経路がアクターへ渡す)、
 *    dyne/node/<id>/range に発行する (pub_every フレームごと)。
 *  - サーボ: LEDC (50Hz、14ビット) でパルスを出し、落ち着いたら止める (hold=false)。
 *
 * ctl 命令 (dyne/ctl/<id> または all。省いた欄は今の値のまま):
 *   {"cmd":"range", "en":true, "probe":false, "res":8, "hz":15, "mode":"auto|cont", "it_ms":10,
 *    "sharp":14, "order":"closest|strongest", "row":3, "flip":true, "pub_every":1,
 *    "sid":0, "x":0.05, "y":0.60, "yaw_deg":45.0, "arm_m":0.0}
 *     row: 送る行 (0..辺-1)。-1 = 全区画 (アクターの CFG_ZONE_ROW は PC が注入する)
 *     flip: 各行の左右を反転 (アクターの区画0 = 左端に合わせる。ULD の並びは左右が逆と言われる。実験0で確認)
 *     sid / x / y / yaw_deg: 首振りなしのときのセンサID と取り付けの位置・向き (部屋の座標、x軸から反時計回り)。
 *       首振りのときは x, y = サーボの軸の位置、yaw_deg = サーボの中央 (center_deg) の向き
 *     arm_m: サーボの軸からセンサの前面までの距離 (センサの位置 = 軸 + arm·(cos 向き, sin 向き))
 *   {"cmd":"pan", "en":true, "gpio":25, "deg":[22.5,...], "true_deg":[...], "id_base":0,
 *    "center_deg":90, "dir":1, "backlash_deg":4, "ms_per_deg":2.5, "settle_ms":150, "frames":3,
 *    "discard":0, "dwell_max_ms":1500, "hold":false, "us_min":500, "us_max":2400, "deg_max":180,
 *    "goto":90}
 *     deg: 各位置のサーボへの指令の角度。true_deg: 較正表 (各位置の実際の向き − 取り付けの向き [度]、
 *       反時計回りが正。分度器などで測った値)。無ければ dir·(deg − center_deg) を使う
 *     goto: 首振りを止めてその角度に向けたまま (取り付けの向きの較正・実験0のサーボ単体の確認用)
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
#include "driver/ledc.h"
#include "vl53l5cx_api.h"

#include "dyne_node.h"
#include "dyne_sense.h"
#include "dyne_timesync.h"

static const char *TAG = "dyne-range";

#define VL53_ADDR 0x29
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_CH    LEDC_CHANNEL_1
#define SERVO_RES   LEDC_TIMER_14_BIT
#define SERVO_HZ    50

/* ==== 設定 ==== */
typedef struct {
    bool en;
    int res, hz, mode, it_ms, sharp, order; /* mode/order: -1 = センサの既定のまま */
    int row;
    bool flip;
    int pub_every;
    uint32_t sid;
    double x, y, yaw_deg, arm_m;
    /* 首振り */
    bool pan_en;
    int gpio;
    dyne_pan_cfg_t pan;
    double center_deg, dir;
    int n_true;
    double true_deg[DYNE_PAN_MAX_POS];
    float goto_deg; /* >= 0 なら首振りを止めてこの角度へ */
} range_cfg_t;

static range_cfg_t s_cfg;          /* 測距のタスクが使う設定 */
static range_cfg_t s_pend;         /* ctl で受けた設定 (タスクが写す) */
static volatile bool s_apply, s_probe;
static SemaphoreHandle_t s_mx;

/* ==== 状態 ==== */
static VL53L5CX_Configuration *s_dev;
static VL53L5CX_ResultsData *s_res;
static bool s_found, s_ranging, s_task_up;
static int s_init_ms = -1;
static int s_nz = 64;
static char s_err[48];
static uint32_t s_seq_raw;     /* センサから読んだフレームの数 */
static uint32_t s_seq;         /* 採ったフレームの数 */
static uint64_t s_read_err;

static portMUX_TYPE s_fmux = portMUX_INITIALIZER_UNLOCKED;
static dyne_range_frame_t s_latest; /* 最新の採ったフレーム (seq = 0 なら無し) */

static dyne_pan_t s_pan;
static bool s_pan_run, s_servo_ok, s_servo_on;
static float s_servo_deg = -1.0f;

static void default_cfg(range_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->en = true;
    c->res = 8;
    c->hz = 15;
    c->mode = -1;
    c->it_ms = -1;
    c->sharp = -1;
    c->order = -1;
    c->row = CONFIG_DYNE_RANGE_ROW;
    c->flip = true;
    c->pub_every = 1;
    c->sid = 0;
    c->pan_en = false;
    c->gpio = CONFIG_DYNE_SERVO_GPIO;
    dyne_pan_default(&c->pan);
    c->center_deg = 90.0;
    c->dir = 1.0;
    c->goto_deg = -1.0f;
}

/* ==== サーボ (LEDC) ==== */

static bool servo_setup(int gpio)
{
    if (gpio < 0) return false;
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RES,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&t) != ESP_OK) return false;
    ledc_channel_config_t ch = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = SERVO_CH,
        .timer_sel = SERVO_TIMER,
        .duty = 0, /* 0 = パルスなし */
        .hpoint = 0,
    };
    return ledc_channel_config(&ch) == ESP_OK;
}

static void servo_pulse(uint32_t us)
{
    uint32_t duty = us == 0 ? 0 : (uint32_t)(((uint64_t)us << 14) / (1000000 / SERVO_HZ));
    /* 次の周期の頭から効く (途中で切った短いパルスを出さない) */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CH);
    s_servo_on = us != 0;
}

static void servo_act(const dyne_pan_act_t *a)
{
    if (!s_servo_ok) return;
    if (a->set) {
        servo_pulse(dyne_pan_pulse_us(&s_cfg.pan, a->deg));
        s_servo_deg = a->deg;
    }
    if (a->off) servo_pulse(0);
}

/* ==== 発行 ==== */

static void publish_state(void)
{
    char sub[32], json[640];
    snprintf(sub, sizeof(sub), "node/%d/sense", dyne_node_id());
    xSemaphoreTake(s_mx, portMAX_DELAY);
    range_cfg_t c = s_cfg;
    xSemaphoreGive(s_mx);
    int n = snprintf(json, sizeof(json),
                     "{\"range\":{\"found\":%s,\"ranging\":%s,\"init_ms\":%d,\"res\":%d,\"hz\":%d,\"row\":%d,"
                     "\"flip\":%s,\"pub_every\":%d,\"sid\":%u,\"x\":%.4f,\"y\":%.4f,\"yaw_deg\":%.4f,"
                     "\"arm_m\":%.4f,\"frames\":%u,\"taken\":%u,\"read_err\":%llu,\"err\":\"%s\"},",
                     s_found ? "true" : "false", s_ranging ? "true" : "false", s_init_ms, c.res, c.hz, c.row,
                     c.flip ? "true" : "false", c.pub_every, (unsigned)c.sid, c.x, c.y, c.yaw_deg, c.arm_m,
                     (unsigned)s_seq_raw, (unsigned)s_seq, (unsigned long long)s_read_err, s_err);
    n += snprintf(json + n, sizeof(json) - n,
                  "\"pan\":{\"en\":%s,\"run\":%s,\"servo\":%s,\"gpio\":%d,\"n_pos\":%d,\"id_base\":%u,"
                  "\"pos\":%d,\"deg\":%.3f,\"cycles\":%llu,\"accept\":%llu,\"discard\":%llu,\"timeout\":%llu}}",
                  c.pan_en ? "true" : "false", s_pan_run ? "true" : "false", s_servo_ok ? "true" : "false",
                  c.gpio, c.pan.n_pos, (unsigned)c.pan.id_base, s_pan.pos, (double)s_servo_deg,
                  (unsigned long long)s_pan.cycles, (unsigned long long)s_pan.n_accept,
                  (unsigned long long)s_pan.n_discard, (unsigned long long)s_pan.n_timeout);
    (void)n;
    dyne_mqtt_pub(sub, json, 0, false, true);
}

static void publish_frame(const char *topic, const dyne_range_frame_t *f, long long step)
{
    static char *json; /* 測距のタスクだけが使う。初回に確保 (静的 RAM を増やさない) */
    enum { JSZ = 1400 };
    if (!json && !(json = malloc(JSZ))) return;
    char sub[40];
    snprintf(sub, sizeof(sub), "node/%d/%s", dyne_node_id(), topic);
    int n = snprintf(json, JSZ,
                     "{\"seq\":%u,\"step\":%lld,\"sid\":%u,\"pos\":%d,\"t_us\":%lld,\"t_loc\":%lld,"
                     "\"count\":%u,\"cols\":%u,\"d_mm\":[",
                     (unsigned)f->seq, step, (unsigned)f->sensor_id, f->pos, (long long)f->t_sync_us,
                     (long long)f->t_local_us, (unsigned)f->count, (unsigned)f->cols);
    for (uint32_t i = 0; i < f->count && n < (int)JSZ - 16; i++)
        n += snprintf(json + n, JSZ - n, "%s%d", i ? "," : "", f->d_mm[i]);
    n += snprintf(json + n, JSZ - n, "],\"st\":[");
    for (uint32_t i = 0; i < f->count && n < (int)JSZ - 8; i++)
        n += snprintf(json + n, JSZ - n, "%s%u", i ? "," : "", f->st[i]);
    if (n < (int)JSZ - 3) snprintf(json + n, JSZ - n, "]}");
    dyne_mqtt_pub(sub, json, 0, false, true);
}

/* node_v2.c から: アクターへ渡したフレームを記録 (PC がシミュレータで同じ入力を再生できるように) */
void dyne_range_publish_fed(const dyne_range_frame_t *f, uint64_t step)
{
    publish_frame("range_fed", f, (long long)step);
}

/* ==== センサ ==== */

static bool sensor_probe_init(void)
{
    i2c_master_bus_handle_t bus = dyne_i2c_bus();
    if (!bus) { snprintf(s_err, sizeof(s_err), "no i2c bus"); return false; }
    if (i2c_master_probe(bus, VL53_ADDR, 50) != ESP_OK) {
        snprintf(s_err, sizeof(s_err), "not found at 0x29");
        return false;
    }
    if (!s_dev) {
        s_dev = calloc(1, sizeof(VL53L5CX_Configuration));
        s_res = calloc(1, sizeof(VL53L5CX_ResultsData));
        if (!s_dev || !s_res) { snprintf(s_err, sizeof(s_err), "alloc"); return false; }
        i2c_device_config_t dc = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = VL53_ADDR,
            .scl_speed_hz = CONFIG_DYNE_RANGE_I2C_HZ,
        };
        if (i2c_master_bus_add_device(bus, &dc, &s_dev->platform.handle) != ESP_OK) {
            snprintf(s_err, sizeof(s_err), "add device");
            return false;
        }
    }
    uint8_t alive = 0;
    if (vl53l5cx_is_alive(s_dev, &alive) || !alive) { snprintf(s_err, sizeof(s_err), "not alive"); return false; }
    int64_t t0 = esp_timer_get_time();
    uint8_t st = vl53l5cx_init(s_dev); /* センサ用のファームの書き込み (約80KB) */
    s_init_ms = (int)((esp_timer_get_time() - t0) / 1000);
    if (st) { snprintf(s_err, sizeof(s_err), "init status=%u", st); return false; }
    s_err[0] = 0;
    ESP_LOGI(TAG, "VL53L5CX ready (init %d ms)", s_init_ms);
    return true;
}

/* 測距の設定を入れ直して始める (止めてから) */
static void sensor_restart(void)
{
    if (!s_found) return;
    if (s_ranging) { vl53l5cx_stop_ranging(s_dev); s_ranging = false; }
    if (!s_cfg.en) return;
    uint8_t st = 0;
    st |= vl53l5cx_set_resolution(s_dev, s_cfg.res == 4 ? VL53L5CX_RESOLUTION_4X4 : VL53L5CX_RESOLUTION_8X8);
    st |= vl53l5cx_set_ranging_frequency_hz(s_dev, (uint8_t)s_cfg.hz);
    if (s_cfg.mode >= 0) st |= vl53l5cx_set_ranging_mode(s_dev, (uint8_t)s_cfg.mode);
    if (s_cfg.it_ms >= 0) st |= vl53l5cx_set_integration_time_ms(s_dev, (uint32_t)s_cfg.it_ms);
    if (s_cfg.sharp >= 0) st |= vl53l5cx_set_sharpener_percent(s_dev, (uint8_t)s_cfg.sharp);
    if (s_cfg.order >= 0) st |= vl53l5cx_set_target_order(s_dev, (uint8_t)s_cfg.order);
    uint8_t nz = 64;
    vl53l5cx_get_resolution(s_dev, &nz); /* 測定中の読み出しは避ける (評価ファームと同じ) */
    s_nz = nz;
    st |= vl53l5cx_start_ranging(s_dev);
    s_ranging = st == 0;
    if (st) snprintf(s_err, sizeof(s_err), "config status=%u", st);
    ESP_LOGI(TAG, "ranging %s: %d zones %d Hz row=%d flip=%d", s_ranging ? "on" : "FAILED", s_nz, s_cfg.hz,
             s_cfg.row, s_cfg.flip);
}

static void pan_restart(void)
{
    dyne_pan_act_t a = { 0 };
    s_pan_run = false;
    if (!s_servo_ok && s_cfg.gpio >= 0 && (s_cfg.pan_en || s_cfg.goto_deg >= 0)) s_servo_ok = servo_setup(s_cfg.gpio);
    if (!s_servo_ok) return;
    s_cfg.pan.frame_period_us = (uint32_t)(1000000 / (s_cfg.hz > 0 ? s_cfg.hz : 15));
    if (s_cfg.goto_deg >= 0) {
        a.set = true;
        a.deg = s_cfg.goto_deg;
        servo_act(&a);
        return;
    }
    if (!s_cfg.pan_en) { servo_pulse(0); return; }
    dyne_pan_start(&s_pan, &s_cfg.pan, s_servo_deg, esp_timer_get_time(), &a);
    servo_act(&a);
    s_pan_run = true;
}

static void handle_frame(int64_t t_ready)
{
    s_seq_raw++;
    int pos = -1;
    uint32_t sid = s_cfg.sid;
    if (s_pan_run) {
        dyne_pan_act_t a;
        pos = dyne_pan_on_frame(&s_pan, t_ready, &a);
        servo_act(&a);
        if (pos < 0) return; /* 動いている間 (積分が動きと重なった) のフレームは捨てる */
        sid = s_cfg.pan.id_base + (uint32_t)pos;
    }
    dyne_range_frame_t f;
    int16_t d[DYNE_RANGE_MAX_ZONES];
    uint8_t st[DYNE_RANGE_MAX_ZONES];
    for (int i = 0; i < s_nz; i++) {
        d[i] = s_res->distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * i];
        st[i] = s_res->target_status[VL53L5CX_NB_TARGET_PER_ZONE * i];
    }
    if (dyne_range_select(d, st, s_nz, s_cfg.row < s_nz / (s_nz == 64 ? 8 : 4) ? s_cfg.row : -1, s_cfg.flip,
                          f.d_mm, f.st, &f.count, &f.cols) != 0)
        return;
    f.sensor_id = sid;
    f.pos = pos;
    f.t_local_us = t_ready;
    f.t_sync_us = dyne_time_at_local(t_ready);
    f.seq = ++s_seq;
    portENTER_CRITICAL(&s_fmux);
    s_latest = f;
    portEXIT_CRITICAL(&s_fmux);
    if (s_cfg.pub_every > 0 && f.seq % (uint32_t)s_cfg.pub_every == 0) publish_frame("range", &f, -1);
}

static void range_task(void *arg)
{
    if (s_cfg.en) {
        s_found = sensor_probe_init();
        sensor_restart();
    }
    publish_state();
    int64_t last_state = esp_timer_get_time();
    TickType_t dly = pdMS_TO_TICKS(3) > 0 ? pdMS_TO_TICKS(3) : 1;
    for (;;) {
        if (s_apply) {
            xSemaphoreTake(s_mx, portMAX_DELAY);
            s_cfg = s_pend;
            s_apply = false;
            xSemaphoreGive(s_mx);
            if (s_probe || (s_cfg.en && !s_found)) {
                s_probe = false;
                if (!s_found) s_found = sensor_probe_init();
            }
            sensor_restart();
            pan_restart();
            publish_state();
        }
        int64_t now = esp_timer_get_time();
        if (s_pan_run) {
            dyne_pan_act_t a;
            dyne_pan_tick(&s_pan, now, &a);
            servo_act(&a);
        }
        if (s_ranging) {
            uint8_t ready = 0;
            if (vl53l5cx_check_data_ready(s_dev, &ready) != 0) s_read_err++;
            if (ready) {
                int64_t t_ready = esp_timer_get_time();
                if (vl53l5cx_get_ranging_data(s_dev, s_res) == 0) handle_frame(t_ready);
                else s_read_err++;
            }
        }
        if (now - last_state > 10000000) { /* 10秒ごとに状態 */
            publish_state();
            last_state = now;
        }
        vTaskDelay(dly);
    }
}

/* ==== 公開の関数 ==== */

void dyne_range_start(void)
{
    if (!s_mx) s_mx = xSemaphoreCreateMutex();
    default_cfg(&s_cfg);
    s_cfg.en = CONFIG_DYNE_RANGE_AUTO;
    s_pend = s_cfg;
    if (!dyne_i2c_bus()) return;
    /* センサの有無にかかわらずタスクは立てる (サーボだけの確認・後から挿したセンサの探し直し) */
    if (xTaskCreate(range_task, "dyne_range", 8192, NULL, 4, NULL) == pdPASS) s_task_up = true;
}

bool dyne_range_active(void)
{
    return s_ranging;
}

bool dyne_range_take(dyne_range_frame_t *out, uint32_t *last_seq, uint32_t *skipped)
{
    bool got = false;
    portENTER_CRITICAL(&s_fmux);
    if (s_latest.seq > *last_seq) {
        *out = s_latest;
        got = true;
    }
    portEXIT_CRITICAL(&s_fmux);
    if (got) {
        *skipped = out->seq - *last_seq - 1;
        if (*last_seq == 0) *skipped = 0;
        *last_seq = out->seq;
    }
    return got;
}

static double heading_deg(const range_cfg_t *c, int k)
{
    if (k < c->n_true) return c->yaw_deg + c->true_deg[k];
    return c->yaw_deg + c->dir * ((double)c->pan.cmd_deg[k] - c->center_deg);
}

int dyne_range_sensors(uint32_t *id, double *x, double *y, double *yaw, int max)
{
    if (!s_mx) return 0;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    range_cfg_t c = s_cfg;
    xSemaphoreGive(s_mx);
    int n = 0;
    int np = c.pan_en ? c.pan.n_pos : 1;
    for (int k = 0; k < np && n < max; k++) {
        double hd = c.pan_en ? heading_deg(&c, k) : c.yaw_deg;
        double th = hd * M_PI / 180.0;
        id[n] = c.pan_en ? c.pan.id_base + (uint32_t)k : c.sid;
        x[n] = c.x + c.arm_m * cos(th);
        y[n] = c.y + c.arm_m * sin(th);
        yaw[n] = th;
        n++;
    }
    return n;
}

int dyne_range_actor_zone_row(void)
{
    return s_cfg.row >= 0 ? 0 : -1;
}

/* ==== ctl ==== */

static void get_num(const cJSON *r, const char *k, double *v)
{
    const cJSON *o = cJSON_GetObjectItem(r, k);
    if (cJSON_IsNumber(o)) *v = o->valuedouble;
}
static void get_int(const cJSON *r, const char *k, int *v)
{
    const cJSON *o = cJSON_GetObjectItem(r, k);
    if (cJSON_IsNumber(o)) *v = o->valueint;
}
static void get_bool(const cJSON *r, const char *k, bool *v)
{
    const cJSON *o = cJSON_GetObjectItem(r, k);
    if (cJSON_IsBool(o)) *v = cJSON_IsTrue(o);
}

static void reply(const char *cmd, const char *err)
{
    char sub[40], json[128];
    snprintf(sub, sizeof(sub), "node/%d/sense/ack", dyne_node_id());
    if (err) snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"ok\":false,\"err\":\"%s\"}", cmd, err);
    else snprintf(json, sizeof(json), "{\"cmd\":\"%s\",\"ok\":true}", cmd);
    dyne_mqtt_pub(sub, json, 1, false, false);
}

bool dyne_range_on_ctl(const char *cmd, const cJSON *root)
{
    bool is_range = strcmp(cmd, "range") == 0, is_pan = strcmp(cmd, "pan") == 0;
    if (!is_range && !is_pan) return false;
    if (!s_mx || !s_task_up) { reply(cmd, "range task not running (no i2c bus?)"); return true; }
    xSemaphoreTake(s_mx, portMAX_DELAY);
    range_cfg_t c = s_apply ? s_pend : s_cfg;
    xSemaphoreGive(s_mx);
    const char *err = NULL;
    if (is_range) {
        bool probe = false;
        get_bool(root, "en", &c.en);
        get_bool(root, "probe", &probe);
        get_int(root, "res", &c.res);
        get_int(root, "hz", &c.hz);
        get_int(root, "it_ms", &c.it_ms);
        get_int(root, "sharp", &c.sharp);
        get_int(root, "row", &c.row);
        get_bool(root, "flip", &c.flip);
        get_int(root, "pub_every", &c.pub_every);
        double sid = c.sid;
        get_num(root, "sid", &sid);
        c.sid = (uint32_t)sid;
        get_num(root, "x", &c.x);
        get_num(root, "y", &c.y);
        get_num(root, "yaw_deg", &c.yaw_deg);
        get_num(root, "arm_m", &c.arm_m);
        const cJSON *m = cJSON_GetObjectItem(root, "mode");
        if (cJSON_IsString(m)) c.mode = !strcmp(m->valuestring, "cont") ? VL53L5CX_RANGING_MODE_CONTINUOUS
                                       : VL53L5CX_RANGING_MODE_AUTONOMOUS;
        const cJSON *o = cJSON_GetObjectItem(root, "order");
        if (cJSON_IsString(o)) c.order = !strcmp(o->valuestring, "closest") ? VL53L5CX_TARGET_ORDER_CLOSEST
                                        : VL53L5CX_TARGET_ORDER_STRONGEST;
        int side = c.res == 4 ? 4 : 8;
        if (c.res != 4 && c.res != 8) err = "res must be 4 or 8";
        else if (c.hz < 1 || c.hz > (c.res == 8 ? 15 : 60)) err = "hz (8x8: 1..15, 4x4: 1..60)";
        else if (c.row < -1 || c.row >= side) err = "row";
        else if (c.sid >= 64) err = "sid must be < 64 (actor MAX_SENSORS)";
        if (probe) s_probe = true;
    } else {
        get_bool(root, "en", &c.pan_en);
        get_int(root, "gpio", &c.gpio);
        double v;
        const cJSON *a = cJSON_GetObjectItem(root, "deg");
        if (cJSON_IsArray(a)) {
            int n = cJSON_GetArraySize(a);
            if (n < 1 || n > DYNE_PAN_MAX_POS) err = "deg: 1..16 positions";
            else {
                c.pan.n_pos = n;
                for (int k = 0; k < n; k++) c.pan.cmd_deg[k] = (float)cJSON_GetArrayItem(a, k)->valuedouble;
                c.n_true = 0; /* 位置を変えたら較正表は入れ直す */
            }
        }
        a = cJSON_GetObjectItem(root, "true_deg");
        if (cJSON_IsArray(a)) {
            int n = cJSON_GetArraySize(a);
            if (n != c.pan.n_pos) err = "true_deg must have one value per position";
            else {
                c.n_true = n;
                for (int k = 0; k < n; k++) c.true_deg[k] = cJSON_GetArrayItem(a, k)->valuedouble;
            }
        }
        v = c.pan.id_base; get_num(root, "id_base", &v); c.pan.id_base = (uint32_t)v;
        get_num(root, "center_deg", &c.center_deg);
        get_num(root, "dir", &c.dir);
        v = c.pan.backlash_deg; get_num(root, "backlash_deg", &v); c.pan.backlash_deg = (float)v;
        v = c.pan.ms_per_deg; get_num(root, "ms_per_deg", &v); c.pan.ms_per_deg = (float)v;
        v = c.pan.settle_ms; get_num(root, "settle_ms", &v); c.pan.settle_ms = (uint32_t)v;
        v = c.pan.frames; get_num(root, "frames", &v); c.pan.frames = (uint32_t)v;
        v = c.pan.discard; get_num(root, "discard", &v); c.pan.discard = (uint32_t)v;
        v = c.pan.dwell_max_ms; get_num(root, "dwell_max_ms", &v); c.pan.dwell_max_ms = (uint32_t)v;
        get_bool(root, "hold", &c.pan.hold);
        v = c.pan.us_min; get_num(root, "us_min", &v); c.pan.us_min = (uint32_t)v;
        v = c.pan.us_max; get_num(root, "us_max", &v); c.pan.us_max = (uint32_t)v;
        v = c.pan.deg_max; get_num(root, "deg_max", &v); c.pan.deg_max = (float)v;
        const cJSON *g = cJSON_GetObjectItem(root, "goto");
        c.goto_deg = cJSON_IsNumber(g) ? (float)g->valuedouble : -1.0f;
        if (c.goto_deg >= 0) c.pan_en = false;
        if (!err) err = dyne_pan_check(&c.pan);
        if (!err && c.pan.id_base + (uint32_t)c.pan.n_pos > 64) err = "id_base + n_pos must be <= 64";
        if (!err && fabs(fabs(c.dir) - 1.0) > 1e-9) err = "dir must be 1 or -1";
        if (!err && s_servo_ok && c.gpio != s_cfg.gpio) err = "gpio cannot change after servo setup (reboot)";
    }
    if (!err) {
        xSemaphoreTake(s_mx, portMAX_DELAY);
        s_pend = c;
        s_apply = true;
        xSemaphoreGive(s_mx);
    }
    reply(cmd, err);
    return true;
}
