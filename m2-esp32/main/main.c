/*
 * Dyne M2: ESP32実機2台のWi-Fi直結による境界同期・協調推論
 *
 * 構成 (docs/2026-08-16-m2-design.md, 接続形態=案b):
 *  - ノードA: アクセスポイント役 (SSID "dyne-m2", 192.168.4.1)。担当 [0, 0.5] 33点
 *  - ノードB: 子機役。担当 [0.5+h, 1.0] 32点
 *  - 役割は menuconfig の "Dyne M2" → "Node role" で選択
 *
 * 実装している仕様 (M0/M1/M-S0からの反映):
 *  - 境界同期: BSPステップ同期、40バイト固定長メッセージ + CRC32 (A-4)
 *  - 界面ID: クラスタ一意 (M0知見3)。2台構成なので界面0のみ。面の読み替えなし
 *  - 送信控え(直近2ステップ)と過去ステップ再要求への応答再送 (M0知見1)
 *  - 再送バックオフ (M1知見4)
 *  - 終了時ドレイン (M0知見2)
 *  - WAMR処理はpthread上で実行 (M1知見3)
 *  - Wi-Fi省電力オフ (遅延計測のため必須)
 *
 * 計測 (限界レポート第4号):
 *  - ステップごとの同期待ち時間の分布 (ウォームアップ100ステップ以降)
 *  - 再送・応答再送回数 (喪失率の推定材料)
 *  - Wi-Fi起動後/インスタンス化後/最小の空きヒープ
 *  - 合否: 温度配列バイト列のCRC32を、PC計算の期待値と照合
 *      期待値: A=0x3CC6AD26, B=0x8AC47DE7 (単一ノード解より事前計算)
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
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include "wasm_export.h"

/* ==== 埋め込みWASM ==== */
extern const uint8_t wasm_start[] asm("_binary_heat1d_actor_wasm_start");
extern const uint8_t wasm_end[]   asm("_binary_heat1d_actor_wasm_end");

/* ==== 役割と問題設定 ==== */
#ifdef CONFIG_DYNE_ROLE_A
#define ROLE_NAME   "A"
#define MY_N        33
#define MY_X0       0.0
#define MY_X1       0.5
#define BC_LEFT     0 /* Dirichlet */
#define BC_RIGHT    1 /* interface */
#define MY_FACE     1 /* 界面0の自分側 = 右面 */
#define CRC_EXPECT  0x3CC6AD26u
#else
#define ROLE_NAME   "B"
#define MY_N        32
#define MY_X0       0.515625
#define MY_X1       1.0
#define BC_LEFT     1
#define BC_RIGHT    0
#define MY_FACE     0 /* 界面0の自分側 = 左面 */
#define CRC_EXPECT  0x8AC47DE7u
#endif

#define IFACE_ID    0      /* クラスタ一意の界面番号 (2台構成では0のみ) */
#define STEPS       2000
#define WARMUP      100    /* 統計から除外する序盤ステップ数 */
#define ALPHA       1.0
#define DT          1e-4

#define WIFI_SSID   "dyne-m2"
#define WIFI_PASS   "changeme-m2"
#define WIFI_CH     6
#define UDP_PORT    7777
#define NODE_A_IP   "192.168.4.1"

#define TIMEOUT_US      100000  /* 返事待ちの上限 (M0と同じ100ms) */
#define RESEND_MIN_US   20000   /* 再送バックオフ (M1知見4) */
#define RECV_SLICE_MS   20      /* 受信の1回あたり待ち時間 */
#define DRAIN_MS        3000    /* 終了時ドレイン */

/* ==== ワイヤ形式 (dyne-protoと同一レイアウト・40バイト) ==== */
#define MAGIC 0x44594E30u
#define WIRE_LEN 40

static uint32_t crc32_ieee(const uint8_t *d, size_t len)
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

static void wire_encode(uint8_t *b, uint16_t iface, uint64_t step, uint32_t seq,
                        double value, double grad)
{
    memcpy(b + 0, &(uint32_t){MAGIC}, 4);
    memcpy(b + 4, &iface, 2);
    memset(b + 6, 0, 2);
    memcpy(b + 8, &step, 8);
    memcpy(b + 16, &seq, 4);
    memcpy(b + 20, &value, 8);
    memcpy(b + 28, &grad, 8);
    uint32_t crc = crc32_ieee(b, 36);
    memcpy(b + 36, &crc, 4);
}

static bool wire_decode(const uint8_t *b, int len, uint16_t *iface,
                        uint64_t *step, double *value, double *grad)
{
    if (len != WIRE_LEN) return false;
    uint32_t magic, crc;
    memcpy(&magic, b + 0, 4);
    if (magic != MAGIC) return false;
    memcpy(&crc, b + 36, 4);
    if (crc32_ieee(b, 36) != crc) return false;
    memcpy(iface, b + 4, 2);
    memcpy(step, b + 8, 8);
    memcpy(value, b + 20, 8);
    memcpy(grad, b + 28, 8);
    return true;
}

/* ==== Wi-Fi ==== */
static EventGroupHandle_t s_ev;
#define EV_GOT_IP BIT0

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("[m2] wifi disconnected, retrying...\n");
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
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
#ifdef CONFIG_DYNE_ROLE_A
    esp_netif_create_default_wifi_ap();
    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, WIFI_SSID);
    strcpy((char *)wc.ap.password, WIFI_PASS);
    wc.ap.ssid_len = strlen(WIFI_SSID);
    wc.ap.channel = WIFI_CH;
    wc.ap.max_connection = 2;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("[m2] role A: access point '%s' up (ch %d)\n", WIFI_SSID, WIFI_CH);
#else
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL));
    wifi_config_t wc = {0};
    strcpy((char *)wc.sta.ssid, WIFI_SSID);
    strcpy((char *)wc.sta.password, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("[m2] role B: connecting to '%s'...\n", WIFI_SSID);
    xEventGroupWaitBits(s_ev, EV_GOT_IP, pdFALSE, pdTRUE, portMAX_DELAY);
    printf("[m2] role B: got IP\n");
#endif
    /* 省電力オフ: 有効のままだと受信が間欠になり遅延が暴れる */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

/* ==== 統計 ==== */
static uint32_t wait_us_log[STEPS];
static char g_report[1024];
static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

/* ==== 本体 (pthread上で実行) ==== */
static void *m2_task(void *arg)
{
    printf("\n==== Dyne M2: 2-node boundary sync (role %s) ====\n", ROLE_NAME);
    wifi_up();
    size_t heap_wifi = esp_get_free_heap_size();
    printf("[m2] free heap after wifi: %u bytes\n", (unsigned)heap_wifi);

    /* ---- WAMR ---- */
    RuntimeInitArgs ia;
    memset(&ia, 0, sizeof(ia));
    ia.mem_alloc_type = Alloc_With_System_Allocator;
    if (!wasm_runtime_full_init(&ia)) { printf("[m2] ERROR: rt init\n"); return NULL; }
    uint32_t wsize = (uint32_t)(wasm_end - wasm_start);
    uint8_t *wbuf = malloc(wsize);
    memcpy(wbuf, wasm_start, wsize);
    char err[128];
    wasm_module_t mod = wasm_runtime_load(wbuf, wsize, err, sizeof(err));
    if (!mod) { printf("[m2] ERROR: load: %s\n", err); return NULL; }
    wasm_module_inst_t inst = wasm_runtime_instantiate(mod, 16 * 1024, 0, err, sizeof(err));
    if (!inst) {
        printf("[m2] ERROR: instantiate: %s\n", err);
        printf("[m2] (メモリ不足なら軽量版wasmへの切替手順を参照)\n");
        return NULL;
    }
    wasm_exec_env_t env = wasm_runtime_create_exec_env(inst, 16 * 1024);
    size_t heap_inst = esp_get_free_heap_size();
    printf("[m2] free heap after instantiate: %u bytes\n", (unsigned)heap_inst);

    wasm_function_inst_t f_init = wasm_runtime_lookup_function(inst, "dyne_init");
    wasm_function_inst_t f_step = wasm_runtime_lookup_function(inst, "dyne_step");
    wasm_function_inst_t f_bval = wasm_runtime_lookup_function(inst, "dyne_boundary_value");
    wasm_function_inst_t f_bgrad = wasm_runtime_lookup_function(inst, "dyne_boundary_grad");
    wasm_function_inst_t f_setn = wasm_runtime_lookup_function(inst, "dyne_set_neighbor");
    wasm_function_inst_t f_get = wasm_runtime_lookup_function(inst, "dyne_get");
    wasm_function_inst_t f_x = wasm_runtime_lookup_function(inst, "dyne_x");
    if (!f_init || !f_step || !f_bval || !f_bgrad || !f_setn || !f_get || !f_x) {
        printf("[m2] ERROR: export lookup\n");
        return NULL;
    }

    {
        wasm_val_t a[7], r[1];
        a[0].kind = WASM_I32; a[0].of.i32 = MY_N;
        a[1].kind = WASM_F64; a[1].of.f64 = MY_X0;
        a[2].kind = WASM_F64; a[2].of.f64 = MY_X1;
        a[3].kind = WASM_F64; a[3].of.f64 = ALPHA;
        a[4].kind = WASM_F64; a[4].of.f64 = DT;
        a[5].kind = WASM_I32; a[5].of.i32 = BC_LEFT;
        a[6].kind = WASM_I32; a[6].of.i32 = BC_RIGHT;
        if (!wasm_runtime_call_wasm_a(env, f_init, 1, r, 7, a) || r[0].of.i32 != 0) {
            printf("[m2] ERROR: dyne_init\n");
            return NULL;
        }
    }

    /* ---- UDP ---- */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in me = {0};
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    me.sin_port = htons(UDP_PORT);
    bind(sock, (struct sockaddr *)&me, sizeof(me));
    struct timeval tv = { .tv_sec = 0, .tv_usec = RECV_SLICE_MS * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in peer = {0};
    bool has_peer = false;
#ifndef CONFIG_DYNE_ROLE_A
    peer.sin_family = AF_INET;
    peer.sin_port = htons(UDP_PORT);
    peer.sin_addr.s_addr = inet_addr(NODE_A_IP);
    has_peer = true; /* Bは相手(A)の固定IPを知っている。AはBの初着信で学習する */
#endif

    /* 送信控え (直近2ステップ) */
    struct { uint64_t step; uint8_t buf[WIRE_LEN]; bool valid; } slot[2] = {0};
    /* 先着した将来ステップ (BSPではずれ最大1なので1枠で足りる) */
    struct { uint64_t step; double v, g; bool valid; } pend = {0};

    uint32_t seq = 0;
    uint64_t stat_sent = 0, stat_resend = 0, stat_replay = 0, stat_crc = 0;

    double bv, bg;
    wasm_val_t va[3], vr[1];

#define READ_BOUNDARY()                                                        \
    do {                                                                       \
        va[0].kind = WASM_I32; va[0].of.i32 = MY_FACE;                         \
        wasm_runtime_call_wasm_a(env, f_bval, 1, vr, 1, va); bv = vr[0].of.f64;\
        wasm_runtime_call_wasm_a(env, f_bgrad, 1, vr, 1, va); bg = vr[0].of.f64;\
    } while (0)

    printf("[m2] starting %d steps...\n", STEPS);
    int64_t t_run0 = esp_timer_get_time();
    int64_t compute_us_total = 0;

    for (uint64_t step = 0; step < STEPS; step++) {
        int64_t t0 = esp_timer_get_time();
        /* 1) 送信 (控えを残す) */
        READ_BOUNDARY();
        seq++;
        int si = (int)(step & 1);
        wire_encode(slot[si].buf, IFACE_ID, step, seq, bv, bg);
        slot[si].step = step;
        slot[si].valid = true;
        int64_t last_send = 0;
        if (has_peer) {
            sendto(sock, slot[si].buf, WIRE_LEN, 0, (struct sockaddr *)&peer, sizeof(peer));
            stat_sent++;
            last_send = esp_timer_get_time();
        }
        /* 2) 相手のstep一致メッセージを待つ */
        bool got = false;
        if (pend.valid && pend.step == step) {
            va[0].kind = WASM_I32; va[0].of.i32 = MY_FACE;
            va[1].kind = WASM_F64; va[1].of.f64 = pend.v;
            va[2].kind = WASM_F64; va[2].of.f64 = pend.g;
            wasm_runtime_call_wasm_a(env, f_setn, 0, NULL, 3, va);
            pend.valid = false;
            got = true;
        }
        while (!got) {
            uint8_t rb[WIRE_LEN];
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            int n = recvfrom(sock, rb, sizeof(rb), 0, (struct sockaddr *)&src, &sl);
            int64_t now = esp_timer_get_time();
            if (n > 0) {
                uint16_t iface; uint64_t mstep; double mv, mg;
                if (!wire_decode(rb, n, &iface, &mstep, &mv, &mg)) {
                    stat_crc++;
                    continue;
                }
                if (iface != IFACE_ID) continue;
                if (!has_peer) {
                    peer = src;
                    has_peer = true;
                    /* 相手を学習したので現ステップぶんを即送信 */
                    sendto(sock, slot[si].buf, WIRE_LEN, 0, (struct sockaddr *)&peer, sizeof(peer));
                    stat_sent++;
                    last_send = now;
                }
                if (mstep == step) {
                    va[0].kind = WASM_I32; va[0].of.i32 = MY_FACE;
                    va[1].kind = WASM_F64; va[1].of.f64 = mv;
                    va[2].kind = WASM_F64; va[2].of.f64 = mg;
                    wasm_runtime_call_wasm_a(env, f_setn, 0, NULL, 3, va);
                    got = true;
                } else if (mstep > step) {
                    pend.step = mstep; pend.v = mv; pend.g = mg; pend.valid = true;
                } else {
                    /* 過去分 = 相手はまだ受け取れていない。控えを再送で応答 */
                    int ri = (int)(mstep & 1);
                    if (slot[ri].valid && slot[ri].step == mstep && has_peer) {
                        sendto(sock, slot[ri].buf, WIRE_LEN, 0,
                               (struct sockaddr *)&peer, sizeof(peer));
                        stat_replay++;
                    }
                }
            } else {
                /* タイムアウトスライス: バックオフを満たしたら現ステップ再送 */
                if (has_peer && last_send != 0
                    && now - last_send >= TIMEOUT_US) {
                    sendto(sock, slot[si].buf, WIRE_LEN, 0,
                           (struct sockaddr *)&peer, sizeof(peer));
                    stat_resend++;
                    last_send = now + RESEND_MIN_US - TIMEOUT_US;
                }
            }
        }
        int64_t t1 = esp_timer_get_time();
        wait_us_log[step] = (uint32_t)(t1 - t0);
        /* 3) 計算 */
        wasm_val_t none[1];
        wasm_runtime_call_wasm_a(env, f_step, 0, none, 0, NULL);
        compute_us_total += esp_timer_get_time() - t1;
        if ((step + 1) % 500 == 0) {
            printf("[m2]   step %llu/%d\n", (unsigned long long)(step + 1), STEPS);
        }
    }
    int64_t t_run1 = esp_timer_get_time();

    /* ---- 終了時ドレイン: 相手の再要求に応答してから締める ---- */
    int64_t drain_end = esp_timer_get_time() + (int64_t)DRAIN_MS * 1000;
    while (esp_timer_get_time() < drain_end) {
        uint8_t rb[WIRE_LEN];
        struct sockaddr_in src; socklen_t sl = sizeof(src);
        int n = recvfrom(sock, rb, sizeof(rb), 0, (struct sockaddr *)&src, &sl);
        if (n > 0) {
            uint16_t iface; uint64_t mstep; double mv, mg;
            if (wire_decode(rb, n, &iface, &mstep, &mv, &mg) && iface == IFACE_ID) {
                int ri = (int)(mstep & 1);
                if (slot[ri].valid && slot[ri].step == mstep && has_peer) {
                    sendto(sock, slot[ri].buf, WIRE_LEN, 0,
                           (struct sockaddr *)&peer, sizeof(peer));
                    stat_replay++;
                }
            }
        }
    }

    /* ---- 検証: CRC32 + 解析解誤差 ---- */
    uint32_t crc = 0xFFFFFFFFu;
    {
        uint8_t bytes[8];
        double l2 = 0.0, linf = 0.0;
        double decay = exp(-ALPHA * M_PI * M_PI * (DT * STEPS));
        uint8_t all[MY_N * 8];
        for (int i = 0; i < MY_N; i++) {
            va[0].kind = WASM_I32; va[0].of.i32 = i;
            wasm_runtime_call_wasm_a(env, f_get, 1, vr, 1, va);
            double u = vr[0].of.f64;
            memcpy(bytes, &u, 8);
            memcpy(all + i * 8, bytes, 8);
            wasm_runtime_call_wasm_a(env, f_x, 1, vr, 1, va);
            double e = fabs(u - sin(M_PI * vr[0].of.f64) * decay);
            l2 += e * e;
            if (e > linf) linf = e;
        }
        crc = crc32_ieee(all, sizeof(all));
        l2 = sqrt(l2 / MY_N);

        /* 待ち時間の分布 (ウォームアップ後) */
        int m = STEPS - WARMUP;
        qsort(wait_us_log + WARMUP, m, sizeof(uint32_t), cmp_u32);
        uint32_t wmin = wait_us_log[WARMUP];
        uint32_t wmed = wait_us_log[WARMUP + m / 2];
        uint32_t wp95 = wait_us_log[WARMUP + (int)(m * 0.95)];
        uint32_t wmax = wait_us_log[STEPS - 1];

        double total_ms = (t_run1 - t_run0) / 1000.0;
        double compute_ms = compute_us_total / 1000.0;
        double per_step_compute = compute_ms / STEPS;

        snprintf(g_report, sizeof(g_report),
                 "==== M2 REPORT (role %s) ====\n"
                 "steps              : %d (warmup %d excluded from wait stats)\n"
                 "free heap wifi/inst: %u / %u bytes\n"
                 "min free heap      : %u bytes\n"
                 "total wall         : %.1f ms\n"
                 "compute            : %.1f ms total, %.2f ms/step\n"
                 "sync wait us       : min=%u med=%u p95=%u max=%u\n"
                 "sync overhead      : %.1f %%\n"
                 "msgs               : sent=%llu resend=%llu replay=%llu crc_err=%llu\n"
                 "L2 / Linf (own dom): %.3e / %.3e\n"
                 "CRC32(u bytes)     : 0x%08X (expect 0x%08X) -> %s\n"
                 "==== END ====\n",
                 ROLE_NAME, STEPS, WARMUP,
                 (unsigned)heap_wifi, (unsigned)heap_inst,
                 (unsigned)esp_get_minimum_free_heap_size(),
                 total_ms, compute_ms, per_step_compute,
                 (unsigned)wmin, (unsigned)wmed, (unsigned)wp95, (unsigned)wmax,
                 (total_ms / compute_ms - 1.0) * 100.0,
                 (unsigned long long)stat_sent, (unsigned long long)stat_resend,
                 (unsigned long long)stat_replay, (unsigned long long)stat_crc,
                 l2, linf,
                 (unsigned)crc, (unsigned)CRC_EXPECT,
                 crc == CRC_EXPECT ? "MATCH" : "MISMATCH");
        fputs("\n", stdout);
        fputs(g_report, stdout);
    }

    /* ---- レポートの無線転送 (USBケーブル1本運用のため) ----
     * ノードBは自分のレポート文をUDPでAへ送り、Aが代理でモニタに表示する。
     * これによりモニタ接続はA側1本で足りる。パケット形式:
     * "DYNR"(4B) + len(u16, リトルエンディアン) + テキスト。喪失に備え反復送信。 */
#ifdef CONFIG_DYNE_ROLE_A
    {
        printf("[m2] waiting for node B report over Wi-Fi (up to 15s)...\n");
        int64_t until = esp_timer_get_time() + 15LL * 1000 * 1000;
        bool shown = false;
        while (esp_timer_get_time() < until && !shown) {
            uint8_t rb[1100];
            struct sockaddr_in src; socklen_t sl = sizeof(src);
            int n = recvfrom(sock, rb, sizeof(rb), 0, (struct sockaddr *)&src, &sl);
            if (n <= 0) continue;
            if (n == WIRE_LEN) {
                /* 走行中メッセージの残骸への応答再送 (念のため継続) */
                uint16_t iface; uint64_t mstep; double mv, mg;
                if (wire_decode(rb, n, &iface, &mstep, &mv, &mg) && iface == IFACE_ID) {
                    int ri = (int)(mstep & 1);
                    if (slot[ri].valid && slot[ri].step == mstep && has_peer) {
                        sendto(sock, slot[ri].buf, WIRE_LEN, 0,
                               (struct sockaddr *)&peer, sizeof(peer));
                    }
                }
                continue;
            }
            if (n > 6 && memcmp(rb, "DYNR", 4) == 0) {
                uint16_t len;
                memcpy(&len, rb + 4, 2);
                if (len <= n - 6 && len < sizeof(rb) - 6) {
                    rb[6 + len] = 0;
                    printf("\n---- report relayed from node B ----\n");
                    fputs((const char *)(rb + 6), stdout);
                    printf("---- end of relayed report ----\n");
                    shown = true;
                }
            }
        }
        if (!shown) {
            printf("[m2] WARN: node B report not received "
                   "(BのUSB接続で直接読む予備手順を参照)\n");
        }
    }
#else
    {
        printf("[m2] sending report to node A over Wi-Fi...\n");
        uint16_t len = (uint16_t)strlen(g_report);
        static uint8_t pkt[1100];
        memcpy(pkt, "DYNR", 4);
        memcpy(pkt + 4, &len, 2);
        memcpy(pkt + 6, g_report, len);
        for (int i = 0; i < 12; i++) {
            if (has_peer) {
                sendto(sock, pkt, 6 + len, 0, (struct sockaddr *)&peer, sizeof(peer));
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        printf("[m2] report sent.\n");
    }
#endif
    return NULL;
}

void app_main(void)
{
    pthread_t t;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 32 * 1024);
    if (pthread_create(&t, &attr, m2_task, NULL) != 0) {
        printf("[m2] ERROR: pthread_create\n");
        return;
    }
    pthread_join(t, NULL);
    printf("[m2] done. idle.\n");
}