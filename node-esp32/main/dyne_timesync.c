/*
 * 時刻同期 (2026-11-11)。宣言と方針は dyne_timesync.h、計算部分は dyne_timesync_core.c。
 *
 * ctl 命令 (dyne/ctl/<id> または dyne/ctl/all。retain 禁止は既存の ctl と同じ):
 *   {"cmd":"timesync",
 *    "server":"192.168.x.1", "port":123,        ← 往復の交換の相手 (既定: ブローカーと同じ PC、123)
 *    "period_ms":2000, "burst":4,               ← 交換の周期と1回あたりの往復数 (最小の往復時間の標本を採る)
 *    "report_every":5,                          ← 何周期ごとに tsync を報告するか (0 = 報告しない)
 *    "pps":true, "pps_gpio":2, "pps_src":"sync"}← 試験モード。pps_src: sync = 同期した時刻 /
 *                                                  sntp = SNTP で合わせたシステムの時計 / local = 同期なし
 * 省略した項目は変えない。
 *
 * 報告 dyne/node/<id>/tsync (QoS0):
 *   {"ts_us":同期した時刻, "synced":bool, "off_us":ずれ, "resid_us":推定との差,
 *    "rtt_us":今回の往復時間, "rtt_min_us":窓の最小, "drift_ppm":ドリフト, "n":窓の標本数,
 *    "acc":採用数, "rej":棄却数, "lost":応答なしの周期数,
 *    "sntp_n":SNTP の同期回数, "sntp_minus_sync_us":システムの時計 - 同期した時刻,
 *    "pps":{"en":bool,"src":..,"gpio":..,"n":切り替え回数,"late_max_us":..,"late_mean_us":..,"skip":..}}
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"

#include "dyne_node.h"
#include "dyne_timesync.h"
#include "dyne_timesync_core.h"

static const char *TAG = "dyne-time";

#define PPS_SRC_SYNC  0
#define PPS_SRC_SNTP  1
#define PPS_SRC_LOCAL 2
static const char *const PPS_SRC_NAME[] = { "sync", "sntp", "local" };

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* 設定 (s_mux で保護) */
static struct {
    char server[64];
    int port, period_ms, burst, report_every;
    bool report_now;
} s_cfg;

/* 推定 (タスクが書き、読み手は s_mux の中で写す) */
static dyne_ts_fit_t s_fit;
static struct {
    int64_t rtt, resid, rtt_min;
    double drift_ppm;
    int n;
    uint32_t acc, rej, lost;
} s_stat;

/* SNTP */
static char s_sntp_server[64];
static volatile uint32_t s_sntp_count;

/* 試験モード */
static portMUX_TYPE s_pps_mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    volatile bool en;
    volatile int gpio, src;
    uint32_t n, skip;
    int64_t late_max, late_sum;
} s_pps = { .gpio = CONFIG_DYNE_PPS_GPIO, .src = PPS_SRC_SYNC };
static TaskHandle_t s_pps_task;
static esp_timer_handle_t s_pps_timer;

/* ==== 公開の読み出し ==== */

static bool fit_copy(dyne_ts_fit_t *f)
{
    portENTER_CRITICAL(&s_mux);
    *f = s_fit;
    portEXIT_CRITICAL(&s_mux);
    return f->ready;
}

bool dyne_time_synced(void)
{
    dyne_ts_fit_t f;
    return fit_copy(&f);
}

int64_t dyne_time_at_local(int64_t local_us)
{
    dyne_ts_fit_t f;
    if (!fit_copy(&f)) return 0;
    return dyne_ts_fit_to_sync(&f, local_us);
}

int64_t dyne_time_now_us(void)
{
    return dyne_time_at_local(esp_timer_get_time());
}

int dyne_timesync_summary(char *buf, int len)
{
    dyne_ts_fit_t f;
    bool ok = fit_copy(&f);
    portENTER_CRITICAL(&s_mux);
    int64_t rtt = s_stat.rtt, resid = s_stat.resid;
    double drift = s_stat.drift_ppm;
    portEXIT_CRITICAL(&s_mux);
    return snprintf(buf, len,
                    "\"tsync\":{\"synced\":%s,\"rtt_us\":%lld,\"resid_us\":%lld,\"drift_ppm\":%.3f,"
                    "\"sntp_n\":%u,\"pps\":%s}",
                    ok ? "true" : "false", (long long)rtt, (long long)resid, drift,
                    (unsigned)s_sntp_count, s_pps.en ? "true" : "false");
}

/* ==== 往復の交換 ==== */

static bool resolve(const char *host, int port, struct sockaddr_in *out)
{
    memset(out, 0, sizeof(*out));
    out->sin_family = AF_INET;
    out->sin_port = htons(port);
    out->sin_addr.s_addr = inet_addr(host);
    if (out->sin_addr.s_addr != IPADDR_NONE) return true;
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM }, *res = NULL;
    if (getaddrinfo(host, NULL, &hints, &res) != 0 || !res) return false;
    out->sin_addr = ((struct sockaddr_in *)res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    return true;
}

/* 往復1回。成功で true */
static bool exchange_once(int sock, const struct sockaddr_in *dst, dyne_ts_sample_t *s, int64_t *mid)
{
    uint8_t pkt[DYNE_NTP_PKT_LEN], rep[64];
    int64_t t1 = esp_timer_get_time();
    dyne_ntp_make_request(pkt, (uint64_t)t1);
    if (sendto(sock, pkt, sizeof(pkt), 0, (const struct sockaddr *)dst, sizeof(*dst)) < 0) return false;
    for (;;) {
        int n = recv(sock, rep, sizeof(rep), 0);
        int64_t t4 = esp_timer_get_time();
        if (n <= 0) return false; /* 受信の待ち時間切れ */
        uint64_t ck;
        int64_t t2, t3;
        if (dyne_ntp_parse_reply(rep, (size_t)n, &ck, &t2, &t3) != 0) continue;
        if (ck != (uint64_t)t1) continue; /* 前の往復への遅れた応答 */
        dyne_ts_sample(t1, t2, t3, t4, s);
        *mid = t1 + (t4 - t1) / 2;
        return true;
    }
}

static void publish_report(int64_t now_local)
{
    dyne_ts_fit_t f;
    bool ok = fit_copy(&f);
    portENTER_CRITICAL(&s_mux);
    __typeof__(s_stat) st = s_stat;
    portEXIT_CRITICAL(&s_mux);
    int64_t sync = ok ? dyne_ts_fit_to_sync(&f, now_local) : 0;
    char sntp_diff[32] = "null";
    if (ok && s_sntp_count > 0) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t l2 = esp_timer_get_time();
        int64_t sys = (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
        snprintf(sntp_diff, sizeof(sntp_diff), "%lld", (long long)(sys - dyne_ts_fit_to_sync(&f, l2)));
    }
    uint32_t pn = s_pps.n;
    char json[512], sub[32];
    snprintf(json, sizeof(json),
             "{\"ts_us\":%lld,\"synced\":%s,\"off_us\":%lld,\"resid_us\":%lld,\"rtt_us\":%lld,"
             "\"rtt_min_us\":%lld,\"drift_ppm\":%.3f,\"n\":%d,\"acc\":%u,\"rej\":%u,\"lost\":%u,"
             "\"sntp_n\":%u,\"sntp_minus_sync_us\":%s,"
             "\"pps\":{\"en\":%s,\"src\":\"%s\",\"gpio\":%d,\"n\":%u,\"late_max_us\":%lld,"
             "\"late_mean_us\":%.1f,\"skip\":%u}}",
             (long long)sync, ok ? "true" : "false", ok ? (long long)dyne_ts_fit_offset(&f, now_local) : 0LL,
             (long long)st.resid, (long long)st.rtt, (long long)st.rtt_min, st.drift_ppm, st.n,
             (unsigned)st.acc, (unsigned)st.rej, (unsigned)st.lost, (unsigned)s_sntp_count, sntp_diff,
             s_pps.en ? "true" : "false", PPS_SRC_NAME[s_pps.src], s_pps.gpio, (unsigned)pn,
             (long long)s_pps.late_max, pn ? (double)s_pps.late_sum / pn : 0.0, (unsigned)s_pps.skip);
    snprintf(sub, sizeof(sub), "node/%d/tsync", dyne_node_id());
    dyne_mqtt_pub(sub, json, 0, false, true);
}

static void tsync_task(void *arg)
{
    static dyne_ts_model_t model; /* 約 800 バイト。タスクのスタックに置かない */
    dyne_ts_model_init(&model);
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct timeval tv = { .tv_sec = 0, .tv_usec = 300 * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint32_t round = 0;
    for (;;) {
        char server[64];
        int port, period, burst, rep_every;
        bool rep_now;
        portENTER_CRITICAL(&s_mux);
        memcpy(server, s_cfg.server, sizeof(server));
        port = s_cfg.port; period = s_cfg.period_ms; burst = s_cfg.burst;
        rep_every = s_cfg.report_every; rep_now = s_cfg.report_now;
        s_cfg.report_now = false;
        portEXIT_CRITICAL(&s_mux);

        struct sockaddr_in dst;
        bool got = false;
        dyne_ts_sample_t best = { 0, INT64_MAX };
        int64_t best_mid = 0;
        if (server[0] && resolve(server, port, &dst)) {
            for (int b = 0; b < burst; b++) {
                dyne_ts_sample_t s;
                int64_t mid;
                if (exchange_once(sock, &dst, &s, &mid) && s.rtt_us >= 0 && s.rtt_us < best.rtt_us) {
                    best = s;
                    best_mid = mid;
                    got = true;
                }
                if (b + 1 < burst) vTaskDelay(pdMS_TO_TICKS(30));
            }
        }
        if (got) {
            int64_t resid;
            dyne_ts_model_add(&model, best_mid, &best, &resid);
            portENTER_CRITICAL(&s_mux);
            s_fit = model.fit;
            s_stat.rtt = best.rtt_us;
            s_stat.resid = resid;
            s_stat.rtt_min = dyne_ts_model_min_rtt(&model);
            s_stat.drift_ppm = model.fit.b * 1e6;
            s_stat.n = model.n;
            s_stat.acc = model.accepted;
            s_stat.rej = model.rejected;
            portEXIT_CRITICAL(&s_mux);
        } else {
            portENTER_CRITICAL(&s_mux);
            s_stat.lost++;
            portEXIT_CRITICAL(&s_mux);
        }
        round++;
        if (dyne_mqtt_connected() && (rep_now || (rep_every > 0 && round % rep_every == 0)))
            publish_report(esp_timer_get_time());
        vTaskDelay(pdMS_TO_TICKS(period > 100 ? period : 100));
    }
}

/* ==== SNTP ==== */

static void on_sntp(struct timeval *tv)
{
    s_sntp_count++;
}

static void sntp_up(void)
{
    /* SNTP の再同期の間隔 (既定の1時間では長すぎる)。下限は 15 秒 */
    esp_sntp_set_sync_interval(15000);
    esp_sntp_config_t c = ESP_NETIF_SNTP_DEFAULT_CONFIG(s_sntp_server);
    c.wait_for_sync = false;
    c.sync_cb = on_sntp;
    if (esp_netif_sntp_init(&c) != ESP_OK) ESP_LOGW(TAG, "sntp init failed");
    else ESP_LOGI(TAG, "sntp server %s", s_sntp_server);
}

/* ==== 試験モード: 整数秒ちょうどにピンを切り替える ==== */

static void pps_timer_cb(void *arg)
{
    xTaskNotifyGive(s_pps_task);
}

/* 次に切り替える時刻 (esp_timer)。label = その整数秒 (偶奇で出力の高低を決め、全ノードの位相をそろえる)。
 * 使えないとき -1 */
static int64_t pps_next(int src, int64_t now, int64_t *label)
{
    const int64_t margin = 3000, P = 1000000;
    if (src == PPS_SRC_SYNC) {
        dyne_ts_fit_t f;
        if (!fit_copy(&f)) return -1;
        int64_t s;
        int64_t t = dyne_ts_fit_next_tick(&f, now, P, margin, &s);
        *label = s / P;
        return t;
    }
    if (src == PPS_SRC_SNTP) {
        if (s_sntp_count == 0) return -1;
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t l = esp_timer_get_time();
        int64_t sys = (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
        int64_t next = ((sys + margin) / P + 1) * P;
        *label = next / P;
        return l + (next - sys);
    }
    int64_t next = ((now + margin) / P + 1) * P;
    *label = next / P;
    return next;
}

static void pps_task(void *arg)
{
    int cur = -1;
    for (;;) {
        if (!s_pps.en || s_pps.gpio < 0) {
            if (cur >= 0) gpio_set_level(cur, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (cur != s_pps.gpio) {
            if (cur >= 0) gpio_reset_pin(cur);
            cur = s_pps.gpio;
            gpio_reset_pin(cur);
            gpio_set_direction(cur, GPIO_MODE_OUTPUT);
            gpio_set_level(cur, 0);
        }
        int64_t now = esp_timer_get_time(), label;
        int64_t target = pps_next(s_pps.src, now, &label);
        if (target < 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        /* 2ms 前にタイマで起こしてもらい、残りは空回りで待つ (FreeRTOS の刻みは 10ms で粗い) */
        int64_t wake = target - 2000 - now;
        esp_timer_start_once(s_pps_timer, wake > 1 ? wake : 1);
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2500));
        if (esp_timer_get_time() > target) { s_pps.skip++; continue; } /* 起こされるのが遅れた */
        while (esp_timer_get_time() < target - 30) { }
        int64_t t;
        portENTER_CRITICAL(&s_pps_mux); /* 最後の 30µs は割り込みを止めて待つ */
        while ((t = esp_timer_get_time()) < target) { }
        gpio_set_level(cur, (int)(label & 1));
        portEXIT_CRITICAL(&s_pps_mux);
        int64_t late = t - target;
        s_pps.n++;
        s_pps.late_sum += late;
        if (late > s_pps.late_max) s_pps.late_max = late;
    }
}

/* ==== 設定 ==== */

static void default_server(char *out, size_t len)
{
    const char *cfg = CONFIG_DYNE_TIME_SERVER;
    if (cfg[0]) {
        strlcpy(out, cfg, len);
        return;
    }
    /* 既定 = ブローカーの URI のホスト部 (mqtt://192.168.x.1:1883 → 192.168.x.1) */
    const char *u = CONFIG_DYNE_BROKER_URI;
    const char *p = strstr(u, "://");
    p = p ? p + 3 : u;
    size_t i = 0;
    while (p[i] && p[i] != ':' && p[i] != '/' && i + 1 < len) { out[i] = p[i]; i++; }
    out[i] = 0;
}

void dyne_timesync_start(void)
{
    default_server(s_cfg.server, sizeof(s_cfg.server));
    strlcpy(s_sntp_server, s_cfg.server, sizeof(s_sntp_server));
    s_cfg.port = CONFIG_DYNE_TIME_PORT;
    s_cfg.period_ms = CONFIG_DYNE_TIME_PERIOD_MS;
    s_cfg.burst = 4;
    s_cfg.report_every = 5;
    sntp_up();
    xTaskCreate(tsync_task, "dyne_tsync", 4096, NULL, 10, NULL);
    const esp_timer_create_args_t ta = { .callback = pps_timer_cb, .name = "dyne_pps" };
    esp_timer_create(&ta, &s_pps_timer);
    /* 最高の優先度で、2コアの ESP32 では Wi-Fi と別のコア (1) に置く */
    xTaskCreatePinnedToCore(pps_task, "dyne_pps", 3072, NULL, configMAX_PRIORITIES - 1, &s_pps_task,
                            portNUM_PROCESSORS > 1 ? 1 : 0);
    ESP_LOGI(TAG, "timesync: server %s:%d period %d ms", s_cfg.server, s_cfg.port, s_cfg.period_ms);
}

void dyne_timesync_on_ctl(const cJSON *root)
{
    const cJSON *v;
    portENTER_CRITICAL(&s_mux);
    if ((v = cJSON_GetObjectItem(root, "server")) && cJSON_IsString(v))
        strlcpy(s_cfg.server, v->valuestring, sizeof(s_cfg.server));
    if ((v = cJSON_GetObjectItem(root, "port")) && cJSON_IsNumber(v) && v->valueint > 0)
        s_cfg.port = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "period_ms")) && cJSON_IsNumber(v) && v->valueint >= 100)
        s_cfg.period_ms = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "burst")) && cJSON_IsNumber(v) && v->valueint >= 1 && v->valueint <= 16)
        s_cfg.burst = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "report_every")) && cJSON_IsNumber(v) && v->valueint >= 0)
        s_cfg.report_every = v->valueint;
    s_cfg.report_now = true;
    portEXIT_CRITICAL(&s_mux);
    if ((v = cJSON_GetObjectItem(root, "pps_gpio")) && cJSON_IsNumber(v)) s_pps.gpio = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "pps_src")) && cJSON_IsString(v)) {
        for (int i = 0; i < 3; i++)
            if (strcmp(v->valuestring, PPS_SRC_NAME[i]) == 0) s_pps.src = i;
    }
    if ((v = cJSON_GetObjectItem(root, "pps"))) {
        bool en = cJSON_IsTrue(v);
        if (en && !s_pps.en) { s_pps.n = 0; s_pps.skip = 0; s_pps.late_max = 0; s_pps.late_sum = 0; }
        s_pps.en = en;
    }
    ESP_LOGI(TAG, "timesync ctl: server %s:%d period %d burst %d pps %d gpio %d src %s", s_cfg.server,
             s_cfg.port, s_cfg.period_ms, s_cfg.burst, s_pps.en, s_pps.gpio, PPS_SRC_NAME[s_pps.src]);
}
