/*
 * 時刻同期の計算部分 (main/dyne_timesync_core.c) を PC 上で確かめる検査 (2026-11-11)。
 *
 * 1. NTP 時刻印 ⇔ Unix µs の往復が µs 単位で一致する
 * 2. 往復1回の式 (ずれ・往復時間) が既知の値を再現する
 * 3. 時計のずれ+水晶の誤差 (ドリフト)+通信の遅延のばらつきを模擬して、推定の誤差が小さい
 * 4. 往復時間の大きな外れ標本を捨てる、PC 側の時計の基準が変わったら推定をやり直す
 * 5. 「次の整数秒」の計算
 * 6. (任意) 引数に PC の時刻サーバのアドレスとポートを渡すと、実際に UDP で往復を交わす
 *    (scripts/dyne_ntp_server.py との組み合わせの確認)
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -Inode-esp32\main ^
 *       node-esp32\host_test\timesync_test.c node-esp32\main\dyne_timesync_core.c ^
 *       -lws2_32 -o %TEMP%\timesync_test.exe
 *   %TEMP%\timesync_test.exe                    → 最終行が "PASS"
 *   %TEMP%\timesync_test.exe 127.0.0.1 12300    → サーバとの往復も確かめる
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dyne_timesync_core.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  NG: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t s_rng = 0x9E3779B97F4A7C15ULL;
static uint64_t rnd(void)
{
    s_rng ^= s_rng << 13; s_rng ^= s_rng >> 7; s_rng ^= s_rng << 17;
    return s_rng;
}
static double urand(void) { return (double)(rnd() >> 11) / 9007199254740992.0; }

static void test_ntp_conv(void)
{
    printf("[1] NTP 時刻印の変換\n");
    int64_t base = 1794700000000000LL; /* 2026-11 頃 */
    int bad = 0;
    for (int i = 0; i < 200000; i++) {
        int64_t us = base + (int64_t)(rnd() % 4000000000000ULL);
        if (dyne_ntp_to_unix_us(dyne_ntp_from_unix_us(us)) != us) bad++;
    }
    CHECK(bad == 0, "roundtrip mismatches=%d", bad);
    /* 既知の値: Unix 0 秒 = NTP 2208988800 秒、0.5 秒 = 端数 0x80000000 */
    CHECK(dyne_ntp_from_unix_us(500000) == ((2208988800ULL << 32) | 0x80000000ULL), "known value");
    uint8_t pkt[DYNE_NTP_PKT_LEN];
    dyne_ntp_make_request(pkt, 0x1122334455667788ULL);
    CHECK(pkt[0] == 0x23, "request first byte 0x%02X", pkt[0]);
    /* 応答を手で作って解釈 */
    uint8_t rep[DYNE_NTP_PKT_LEN] = { 0 };
    rep[0] = 0x24; rep[1] = 1;
    memcpy(rep + 24, pkt + 40, 8);
    uint64_t r2 = dyne_ntp_from_unix_us(base + 10), r3 = dyne_ntp_from_unix_us(base + 25);
    for (int k = 0; k < 8; k++) { rep[32 + k] = (uint8_t)(r2 >> (56 - 8 * k)); rep[40 + k] = (uint8_t)(r3 >> (56 - 8 * k)); }
    uint64_t ck; int64_t t2, t3;
    CHECK(dyne_ntp_parse_reply(rep, sizeof(rep), &ck, &t2, &t3) == 0, "parse ok");
    CHECK(ck == 0x1122334455667788ULL && t2 == base + 10 && t3 == base + 25, "parse values");
    rep[1] = 0;
    CHECK(dyne_ntp_parse_reply(rep, sizeof(rep), &ck, &t2, &t3) == -3, "stratum 0 rejected");
    rep[1] = 1; rep[0] = 0x23;
    CHECK(dyne_ntp_parse_reply(rep, sizeof(rep), &ck, &t2, &t3) == -2, "client mode rejected");
}

static void test_sample(void)
{
    printf("[2] 往復1回の式\n");
    /* サーバの時計 = ノードの時計 + 1000000。行き 300µs、サーバ内 50µs、帰り 500µs */
    dyne_ts_sample_t s;
    int64_t t1 = 5000, t2 = t1 + 300 + 1000000, t3 = t2 + 50, t4 = (t3 - 1000000) + 500;
    dyne_ts_sample(t1, t2, t3, t4, &s);
    CHECK(s.rtt_us == 800, "rtt=%lld", (long long)s.rtt_us);
    /* 行きと帰りの差の半分 (100µs) だけずれる: NTP の式の既知の限界 */
    CHECK(s.offset_us == 1000000 - 100, "offset=%lld", (long long)s.offset_us);
}

/* 模擬: 真のずれ off(t) = O0 + drift*t。往復4回の中から往復時間が最小のものを採る (ファームと同じ) */
typedef struct { double err_max, err_rms, drift_err_ppm; int n_eval; } simres_t;

static simres_t simulate(int64_t O0, double drift, double d_min, double d_jit, int rounds,
                         int64_t period_us, int inject_step_at, int64_t step_us)
{
    dyne_ts_model_t m;
    dyne_ts_model_init(&m);
    simres_t r = { 0 };
    double se = 0;
    int64_t t = 3000000;
    for (int k = 0; k < rounds; k++) {
        if (k == inject_step_at) O0 += step_us;
        dyne_ts_sample_t best = { 0, 1LL << 60 };
        int64_t best_mid = 0;
        for (int b = 0; b < 4; b++) {
            int64_t t1 = t + b * 50000;
            double up = d_min + d_jit * urand() * urand() * 4; /* 片側に長い裾 */
            double dn = d_min + d_jit * urand() * urand() * 4;
            double t2s = (double)t1 + up;            /* ノードの時計での到着 */
            int64_t t2 = (int64_t)llround(t2s + O0 + drift * t2s);
            int64_t t3 = t2 + 40;
            double t3l = t2s + 40;
            int64_t t4 = (int64_t)llround(t3l + dn);
            dyne_ts_sample_t s;
            dyne_ts_sample(t1, t2, t3, t4, &s);
            if (s.rtt_us < best.rtt_us) { best = s; best_mid = t1 + (t4 - t1) / 2; }
        }
        int64_t resid;
        dyne_ts_model_add(&m, best_mid, &best, &resid);
        /* 評価: 立ち上がり (30 標本) の後、次の標本までの途中の時刻での予測誤差 */
        if (k >= 30 && !(inject_step_at >= 0 && k >= inject_step_at && k < inject_step_at + 30)) {
            for (int q = 1; q <= 4; q++) {
                int64_t te = t + period_us * q / 4;
                double truth = (double)O0 + drift * (double)te;
                double e = fabs((double)dyne_ts_fit_offset(&m.fit, te) - truth);
                if (e > r.err_max) r.err_max = e;
                se += e * e;
                r.n_eval++;
            }
        }
        t += period_us;
    }
    r.err_rms = r.n_eval ? sqrt(se / r.n_eval) : 0;
    r.drift_err_ppm = fabs(m.fit.b - drift) * 1e6;
    return r;
}

static void test_model(void)
{
    printf("[3] 推定の精度 (模擬)\n");
    int64_t O0 = 1794700000000000LL;
    simres_t a = simulate(O0, 25e-6, 1000, 50, 200, 2000000, -1, 0);
    printf("  揺らぎ小 (片道1ms+0〜200µs): 最大誤差 %.1f µs, 二乗平均 %.1f µs, ドリフト誤差 %.2f ppm\n",
           a.err_max, a.err_rms, a.drift_err_ppm);
    CHECK(a.err_max < 30, "low-jitter err_max=%.1f", a.err_max);
    CHECK(a.drift_err_ppm < 1.0, "low-jitter drift err=%.2f", a.drift_err_ppm);
    simres_t b = simulate(O0, -40e-6, 1500, 1500, 300, 2000000, -1, 0);
    printf("  揺らぎ大 (片道1.5ms+0〜6ms の裾): 最大誤差 %.1f µs, 二乗平均 %.1f µs, ドリフト誤差 %.2f ppm\n",
           b.err_max, b.err_rms, b.drift_err_ppm);
    CHECK(b.err_rms < 300, "high-jitter err_rms=%.1f", b.err_rms);
    CHECK(b.drift_err_ppm < 10.0, "high-jitter drift err=%.2f", b.drift_err_ppm);

    printf("[4] 外れ標本と基準の変化\n");
    simres_t c = simulate(O0, 10e-6, 1000, 50, 200, 2000000, 100, 1000000);
    printf("  途中で PC の時計が 1 秒跳ぶ: 回復後の最大誤差 %.1f µs\n", c.err_max);
    CHECK(c.err_max < 30, "after step err_max=%.1f", c.err_max);

    dyne_ts_model_t m;
    dyne_ts_model_init(&m);
    for (int k = 0; k < 10; k++) {
        dyne_ts_sample_t s = { O0, 2000 };
        dyne_ts_model_add(&m, 1000000LL * k, &s, NULL);
    }
    dyne_ts_sample_t bad = { O0 + 25000, 50000 };
    int64_t resid;
    int acc = dyne_ts_model_add(&m, 11000000, &bad, &resid);
    CHECK(acc == 0, "outlier accepted");
    CHECK(resid == 25000, "outlier resid=%lld", (long long)resid);
    CHECK(dyne_ts_fit_offset(&m.fit, 11000000) == O0, "fit moved by outlier");
    for (int k = 0; k < 4; k++) dyne_ts_model_add(&m, 12000000 + k, &bad, NULL);
    CHECK(m.accepted == 11, "persistent change re-accepted (accepted=%u)", m.accepted);
}

static void test_tick(void)
{
    printf("[5] 次の整数秒\n");
    dyne_ts_fit_t f = { true, 1794700000123456LL, 7000000, 321.0, 33e-6 };
    int bad = 0;
    for (int i = 0; i < 100000; i++) {
        int64_t now = 1000000 + (int64_t)(rnd() % 100000000000ULL);
        int64_t sync;
        int64_t tl = dyne_ts_fit_next_tick(&f, now, 1000000, 2000, &sync);
        int64_t back = dyne_ts_fit_to_sync(&f, tl);
        if (sync % 1000000 != 0 || llabs(back - sync) > 1 || tl < now + 2000 - 1 || tl > now + 2000 + 1000001) bad++;
    }
    CHECK(bad == 0, "next_tick bad=%d", bad);
}

/* ==== (任意) PC の時刻サーバとの実際の往復 ==== */
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
static int64_t local_us(void)
{
    static LARGE_INTEGER fq;
    LARGE_INTEGER c;
    if (!fq.QuadPart) QueryPerformanceFrequency(&fq);
    QueryPerformanceCounter(&c);
    return (int64_t)((double)c.QuadPart * 1e6 / (double)fq.QuadPart);
}
static int live(const char *host, int port)
{
    printf("[6] サーバ %s:%d との往復\n", host, port);
    WSADATA w;
    WSAStartup(MAKEWORD(2, 2), &w);
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    DWORD to = 500;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&to, sizeof(to));
    struct sockaddr_in a = { 0 };
    a.sin_family = AF_INET;
    a.sin_port = htons((u_short)port);
    inet_pton(AF_INET, host, &a.sin_addr);
    int ok = 0;
    dyne_ts_model_t m;
    dyne_ts_model_init(&m);
    for (int i = 0; i < 20; i++) {
        uint8_t pkt[DYNE_NTP_PKT_LEN], rep[128];
        int64_t t1 = local_us();
        dyne_ntp_make_request(pkt, (uint64_t)t1);
        sendto(s, (const char *)pkt, sizeof(pkt), 0, (struct sockaddr *)&a, sizeof(a));
        int n = recv(s, (char *)rep, sizeof(rep), 0);
        int64_t t4 = local_us();
        uint64_t ck; int64_t t2, t3;
        if (n > 0 && dyne_ntp_parse_reply(rep, (size_t)n, &ck, &t2, &t3) == 0 && ck == (uint64_t)t1) {
            dyne_ts_sample_t smp;
            dyne_ts_sample(t1, t2, t3, t4, &smp);
            int64_t resid;
            dyne_ts_model_add(&m, t1 + (t4 - t1) / 2, &smp, &resid);
            if (i % 5 == 0)
                printf("  #%d rtt=%lld us  offset=%lld us  resid=%lld us\n", i, (long long)smp.rtt_us,
                       (long long)smp.offset_us, (long long)resid);
            ok++;
        }
        Sleep(20);
    }
    closesocket(s);
    printf("  応答 %d/20, 最小往復 %lld us\n", ok, (long long)dyne_ts_model_min_rtt(&m));
    CHECK(ok >= 18, "live replies=%d", ok);
    return 0;
}
#else
static int live(const char *host, int port) { (void)host; (void)port; printf("[6] (Windows 以外は省略)\n"); return 0; }
#endif

int main(int argc, char **argv)
{
    test_ntp_conv();
    test_sample();
    test_model();
    test_tick();
    if (argc >= 3) live(argv[1], atoi(argv[2]));
    printf("%s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
