/*
 * センサまわりの計算部分 (main/dyne_sense_core.c) を PC 上で確かめる検査 (2026-11-13)。
 *
 * 1. RANGE_FRAME の組み立てが Rust (crates/dyne-wire の encode、dyne-sim2d の frame_unit と同じ組み立て)
 *    とバイト単位で一致する (照合用バイト列は Rust で生成した。生成の手順は下の GOLDEN の注記)
 * 2. 区画の行の取り出しと左右の反転
 * 3. 首振りの手順: 位置の順、常に小さい側から近づける、動いている間のフレームを捨てる、
 *    落ち着いたら PWM を止める、フレームが来なくても次へ進む
 * 4. IMU の FIFO の語の解釈 (時刻印 → 値の並び、桁あふれ)
 * 5. センサの時刻印 → esp_timer の当てはめ (発振の誤差 1.2% と読み出しの揺らぎを与える)
 * 6. (2026-11-13 追記) IMU の融合の実行: 開始の時刻 t0 の自動の決め方、imu_fused の JSON
 *    (引数にファイル名を渡すと JSON を書き出す。shape と同じく Python の json.loads で読めるかを
 *     imu_fused_check.py で確かめる。検査の値を C と Python で同じにするため -ffp-contract=off でビルドする)
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -ffp-contract=off -Inode-esp32\main ^
 *       node-esp32\host_test\sense_test.c node-esp32\main\dyne_sense_core.c -o %TEMP%\sense_test.exe
 *   %TEMP%\sense_test.exe %TEMP%\imu_fused.json   → 最終行が "PASS"
 *   .venv\Scripts\python.exe node-esp32\host_test\imu_fused_check.py %TEMP%\imu_fused.json   → "PASS"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "dyne_sense_core.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  NG: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* Rust で生成: dyne_wire::encode(Header{kind: RANGE_FRAME, quantity: QTY_NONE, src: 17, step: 42,
 * t_us: 1731456789123456}, RangeFrame{count: 8, cols: 8}) を枠 ([u32 長さ]) に入れたもの。
 * 区画の距離 [mm] = {0,1,250,999,1234,3999,4000,-7} を /1000.0、状態値 = {5,9,255,4,5,0,9,5} */
static const uint8_t GOLDEN[164] = {
0xa0, 0x00, 0x00, 0x00, 0x02, 0x04, 0xff, 0x00, 0x11, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0x11, 0xdc, 0x32, 0xc0, 0x26, 0x06, 0x00, 0x08, 0x00, 0x00, 0x00,
0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xfc, 0xa9, 0xf1, 0xd2, 0x4d, 0x62, 0x50, 0x3f, 0x09, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0x3f, 0xff, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x2b, 0x87, 0x16, 0xd9, 0xce, 0xf7, 0xef, 0x3f, 0x04, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x58, 0x39, 0xb4, 0xc8, 0x76, 0xbe, 0xf3, 0x3f, 0x05, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xcb, 0xa1, 0x45, 0xb6, 0xf3, 0xfd, 0x0f, 0x40, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x40, 0x09, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x79, 0xe9, 0x26, 0x31, 0x08, 0xac, 0x7c, 0xbf, 0x05, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
};

static void test_range_frame(void)
{
    printf("[1] RANGE_FRAME vs Rust golden\n");
    const int16_t mm[8] = { 0, 1, 250, 999, 1234, 3999, 4000, -7 };
    const uint8_t st[8] = { 5, 9, 255, 4, 5, 0, 9, 5 };
    uint8_t out[256];
    int n = dyne_range_frame_build(out, sizeof(out), 17, 42, 1731456789123456ull, mm, st, 8, 8);
    CHECK(n == (int)sizeof(GOLDEN), "len %d", n);
    CHECK(n == (int)sizeof(GOLDEN) && memcmp(out, GOLDEN, sizeof(GOLDEN)) == 0, "bytes differ");
    CHECK(dyne_range_frame_build(out, 163, 17, 42, 0, mm, st, 8, 8) == -1, "cap");
    CHECK(dyne_range_frame_build(out, sizeof(out), 17, 42, 0, mm, st, 8, 3) == -1, "cols");
}

static void test_select(void)
{
    printf("[2] zone select / flip\n");
    int16_t d[64];
    uint8_t s[64];
    for (int i = 0; i < 64; i++) { d[i] = (int16_t)(100 + i); s[i] = (uint8_t)i; }
    int16_t od[64];
    uint8_t os[64];
    uint32_t cnt, cols;
    CHECK(dyne_range_select(d, s, 64, 3, false, od, os, &cnt, &cols) == 0 && cnt == 8 && cols == 8, "row3");
    for (int c = 0; c < 8; c++) CHECK(od[c] == 100 + 24 + c && os[c] == 24 + c, "row3 c%d", c);
    CHECK(dyne_range_select(d, s, 64, 3, true, od, os, &cnt, &cols) == 0, "row3 flip");
    for (int c = 0; c < 8; c++) CHECK(od[c] == 100 + 24 + 7 - c, "flip c%d", c);
    CHECK(dyne_range_select(d, s, 64, -1, true, od, os, &cnt, &cols) == 0 && cnt == 64 && cols == 8, "all");
    CHECK(od[8] == 100 + 15 && od[63] == 100 + 56, "all flip");
    CHECK(dyne_range_select(d, s, 16, 1, false, od, os, &cnt, &cols) == 0 && cnt == 4 && cols == 4 && od[0] == 104, "4x4");
    CHECK(dyne_range_select(d, s, 64, 8, false, od, os, &cnt, &cols) == -1, "row range");
    CHECK(dyne_range_select(d, s, 32, 0, false, od, os, &cnt, &cols) == -1, "nz");
}

static void test_pan(void)
{
    printf("[3] pan schedule\n");
    dyne_pan_cfg_t c;
    dyne_pan_default(&c);
    CHECK(dyne_pan_check(&c) == NULL, "default cfg");
    CHECK(dyne_pan_pulse_us(&c, 0) == 500 && dyne_pan_pulse_us(&c, 180) == 2400 && dyne_pan_pulse_us(&c, 90) == 1450,
          "pulse");
    dyne_pan_t p;
    dyne_pan_act_t a;
    int64_t now = 1000000;
    /* 角度が不明 → 手前 (22.5−4 = 18.5) へ寄せてから 22.5 へ */
    dyne_pan_start(&p, &c, -1.0f, now, &a);
    CHECK(a.set && fabsf(a.deg - 18.5f) < 1e-4f && p.state == DYNE_PAN_PRE, "pre %f", a.deg);
    int64_t frame_t = now;
    int order[32], n_order = 0;
    float last_cmd = a.deg;
    int approach_bad = 0, offs = 0;
    float prev_set = a.deg;
    /* 15Hz のフレームと 1ms 刻みの時間で 3 巡ぶん回す */
    for (int64_t t = now; t < now + 40000000 && n_order < 24; t += 1000) {
        dyne_pan_tick(&p, t, &a);
        if (a.set) {
            /* 位置の指令 (手前の指令でない) は、直前の指令より大きい角度からは来ない */
            if (p.state == DYNE_PAN_MOVE && a.deg < prev_set - 1e-4f) approach_bad++;
            prev_set = a.deg;
            last_cmd = a.deg;
        }
        if (a.off) offs++;
        if (t - frame_t >= 66667) {
            frame_t = t;
            bool was_dwell = p.state == DYNE_PAN_DWELL;
            int64_t done = p.t_done;
            int pos = dyne_pan_on_frame(&p, t, &a);
            if (pos >= 0) {
                CHECK(was_dwell && t - 66667 >= done, "accepted overlapping frame");
                if (n_order == 0 || order[n_order - 1] != pos || p.taken == 1) order[n_order++] = pos;
            }
            if (a.set) {
                if (p.state == DYNE_PAN_MOVE && a.deg < prev_set - 1e-4f) approach_bad++;
                prev_set = a.deg;
                last_cmd = a.deg;
            }
        }
    }
    (void)last_cmd;
    CHECK(approach_bad == 0, "approached from above %d times", approach_bad);
    for (int i = 0; i < n_order; i++) CHECK(order[i] == i % 8, "order[%d]=%d", i, order[i]);
    CHECK(p.cycles >= 2, "cycles %llu", (unsigned long long)p.cycles);
    CHECK(offs >= 16, "pwm off count %d", offs);
    CHECK(p.n_accept >= 24 * 3 - 3, "accept %llu", (unsigned long long)p.n_accept);
    CHECK(p.n_discard > 0, "no discard");
    printf("    cycles=%llu accept=%llu discard=%llu timeout=%llu\n", (unsigned long long)p.cycles,
           (unsigned long long)p.n_accept, (unsigned long long)p.n_discard, (unsigned long long)p.n_timeout);

    /* フレームが来ない (センサなし) → 1位置の最長の滞在で次へ進む */
    dyne_pan_start(&p, &c, 90.0f, 0, &a);
    for (int64_t t = 0; t < 20000000; t += 1000) dyne_pan_tick(&p, t, &a);
    CHECK(p.n_timeout >= 8 && p.cycles >= 1, "timeout advance %llu", (unsigned long long)p.n_timeout);

    /* 設定の検査 */
    dyne_pan_cfg_t bad = c;
    bad.cmd_deg[3] = 200.0f;
    CHECK(dyne_pan_check(&bad) != NULL, "bad deg accepted");
    bad = c;
    bad.n_pos = 0;
    CHECK(dyne_pan_check(&bad) != NULL, "bad n_pos accepted");
}

static void word(uint8_t w[7], int tag, const uint8_t *d6)
{
    w[0] = (uint8_t)(tag << 3);
    memcpy(w + 1, d6, 6);
}

static void test_imu_parse(void)
{
    printf("[4] IMU FIFO words\n");
    dyne_imu_parse_t p;
    dyne_imu_parse_init(&p);
    dyne_imu_raw_t r;
    uint8_t w[7];
    uint32_t ts = 0xFFFFFF00u;
    int got = 0;
    uint64_t last_tick = 0;
    for (int i = 0; i < 10; i++) {
        uint8_t d[6] = { (uint8_t)ts, (uint8_t)(ts >> 8), (uint8_t)(ts >> 16), (uint8_t)(ts >> 24), 0, 0 };
        word(w, DYNE_LSM_TAG_TS, d);
        CHECK(dyne_imu_fifo_word(&p, w, &r) == 0, "ts emits");
        uint8_t g[6] = { 1, 0, 0xFE, 0xFF, (uint8_t)i, 0 };      /* 1, -2, i */
        uint8_t a[6] = { 0x00, 0x40, 0x00, 0xC0, 0x10, 0x00 };   /* 16384, -16384, 16 */
        word(w, DYNE_LSM_TAG_GYRO, g);
        CHECK(dyne_imu_fifo_word(&p, w, &r) == 0, "gyro alone emits");
        word(w, DYNE_LSM_TAG_ACCEL, a);
        if (dyne_imu_fifo_word(&p, w, &r) == 1) {
            got++;
            CHECK(r.g[0] == 1 && r.g[1] == -2 && r.g[2] == i, "gyro values");
            CHECK(r.a[0] == 16384 && r.a[1] == -16384 && r.a[2] == 16, "acc values");
            CHECK(r.tick > last_tick, "tick monotonic (wrap) %llu", (unsigned long long)r.tick);
            last_tick = r.tick;
        }
        ts += 0x40; /* 4 回目で 2^32 を越える */
    }
    CHECK(got == 10, "samples %d", got);
    CHECK(last_tick == 0xFFFFFF00ull + 9 * 0x40, "unwrapped %llx", (unsigned long long)last_tick);
    CHECK(p.n_orphan == 0, "orphan");
    /* 時刻印の無い値は捨てて数える */
    dyne_imu_parse_init(&p);
    uint8_t z[6] = { 0 };
    word(w, DYNE_LSM_TAG_GYRO, z);
    dyne_imu_fifo_word(&p, w, &r);
    word(w, DYNE_LSM_TAG_ACCEL, z);
    CHECK(dyne_imu_fifo_word(&p, w, &r) == 0 && p.n_orphan == 1, "orphan counted");

    CHECK(dyne_lsm_odr_code(104) == 4 && dyne_lsm_odr_code(208) == 5 && dyne_lsm_odr_code(100) == -1, "odr");
    CHECK(dyne_lsm_fs_xl_bits(4) == 2 && dyne_lsm_fs_g_bits(500) == 1, "fs bits");
    CHECK(fabs(dyne_lsm_us_per_tick(0) - 25.0) < 1e-12, "tick 25us");
}

static uint64_t s_rng = 0x9E3779B97F4A7C15ULL;
static double urand(void)
{
    s_rng ^= s_rng << 13; s_rng ^= s_rng >> 7; s_rng ^= s_rng << 17;
    return (double)(s_rng >> 11) / 9007199254740992.0;
}

static void test_imu_clock(void)
{
    printf("[5] IMU tick -> esp_timer fit\n");
    /* 実際の1刻み 25.3µs (公称 25µs から +1.2%)。FREQ_FINE で公称を 25.3 に直せる場合と、直さない場合 */
    for (int pass = 0; pass < 2; pass++) {
        double true_rate = 25.3, nominal = pass == 0 ? 25.0 : 25.3;
        dyne_imu_clock_t c;
        dyne_imu_clock_init(&c, nominal);
        int64_t boot_off = 12345678; /* tick 0 の esp_timer の時刻 */
        double maxerr = 0;
        for (int i = 0; i < 400; i++) {
            /* 50ms ごとに時刻印のレジスタを読む。読み出しの時刻の揺らぎ ±150µs */
            int64_t loc_true = boot_off + (int64_t)i * 50000 + 7;
            uint64_t tick = (uint64_t)((loc_true - boot_off) / true_rate);
            int64_t loc_meas = loc_true + (int64_t)((urand() - 0.5) * 300.0);
            dyne_imu_clock_add(&c, tick, loc_meas);
            if (i >= 40) {
                /* 直前の 50ms の中の標本の時刻を予測 */
                uint64_t tq = tick - 1000;
                double truth = boot_off + tq * true_rate;
                double e = fabs((double)dyne_imu_clock_local(&c, tq) - truth);
                if (e > maxerr) maxerr = e;
            }
        }
        printf("    nominal=%.1f fitted=%.4f maxerr=%.1fus resid_max=%.1fus\n", nominal, c.rate, maxerr, c.resid_max);
        CHECK(fabs(c.rate - true_rate) < 0.01, "rate %f", c.rate);
        CHECK(maxerr < 80.0, "maxerr %f", maxerr);
    }
    dyne_imu_clock_t c0;
    dyne_imu_clock_init(&c0, 25.0);
    CHECK(dyne_imu_clock_local(&c0, 100) == 0, "empty clock");
}

static void test_imu_run(const char *json_out)
{
    printf("[6] IMU fusion run helpers\n");
    CHECK(dyne_imu_auto_t0(1731456789123456ll, 3000000, 10000000) == 1731456800000000ll, "auto t0");
    CHECK(dyne_imu_auto_t0(1731456797000000ll, 3000000, 10000000) == 1731456800000000ll, "auto t0 exact");
    CHECK(dyne_imu_auto_t0(1731456797000001ll, 3000000, 10000000) == 1731456810000000ll, "auto t0 next");
    double v[22];
    for (int i = 0; i < 22; i++) v[i] = i * 0.1 - 1.0;
    v[0] = 41.0;
    v[3] = 9.80665;
    v[8] = NAN;
    v[9] = INFINITY;
    v[10] = -INFINITY;
    v[11] = 1e-300;
    char js[1024];
    int n = dyne_imu_fused_json(js, sizeof(js), 420, v, 22);
    CHECK(n > 0 && strncmp(js, "{\"step\":420,\"v\":[41,", 20) == 0, "json head %s", js);
    CHECK(strstr(js, ",NaN,Infinity,-Infinity,") != NULL, "non-finite tokens");
    CHECK(dyne_imu_fused_json(js, 40, 420, v, 22) == -1, "cap");
    n = dyne_imu_fused_json(js, sizeof(js), 420, v, 22);
    if (json_out) {
        FILE *f = fopen(json_out, "wb");
        if (f) { fwrite(js, 1, (size_t)n, f); fclose(f); }
    }
}

int main(int argc, char **argv)
{
    test_imu_run(argc > 1 ? argv[1] : NULL);
    test_range_frame();
    test_select();
    test_pan();
    test_imu_parse();
    test_imu_clock();
    printf("%s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
