/*
 * 協調して動かしたときの記録の部品 (main/dyne_perf_core.c) を PC 上で確かめる検査 (2026-11-26、GitHub issue #167)。
 *
 * 1. 記録の追加と、領域があふれたときの数え方
 * 2. 1件の JSON (crates/dyne-sim2d の --perf-log と同じキーと並び)
 * 3. 配送単位のヘッダの時刻印の読み出し
 * 4. まとめて送る形 (小さな領域に分けても、全件がちょうど1回ずつ入る)
 * 引数 --emit: 2台が 100 ステップ交換したことにした記録を、MQTT の記録 (scripts/hw_replay.py record と同じ形、
 *   1行 = {"t","topic","payload"}) で標準出力に書く。遅れは 3, 4, 5, 6, 7 ms の繰り返し (中央値 5 ms)。
 *   scripts/coop_perf.py --from-mqtt が読み戻せることを CI ([24]) で確かめる。
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   cc -std=c11 -Wall -Wextra -O2 -Inode-esp32/main node-esp32/host_test/perf_test.c node-esp32/main/dyne_perf_core.c -o perf_test
 *   ./perf_test          → 最終行が "PASS"
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "dyne_perf_core.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  NG: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void test_push(void)
{
    printf("[1] 記録の追加とあふれ\n");
    dyne_perf_rec_t mem[3];
    dyne_perf_t p;
    dyne_perf_init(&p, mem, 3);
    dyne_perf_tx(&p, 5, 1, 100, 204, false);
    dyne_perf_rx(&p, 5, 1, 100, 160, 204);
    dyne_perf_step(&p, 1, 90, 170, 250);
    dyne_perf_tx(&p, 5, 2, 300, 204, true);
    CHECK(p.n == 3 && p.dropped == 1, "n %u dropped %u", (unsigned)p.n, (unsigned)p.dropped);
    CHECK(sizeof(dyne_perf_rec_t) == 40, "record size %u", (unsigned)sizeof(dyne_perf_rec_t));
    dyne_perf_t off;
    dyne_perf_init(&off, NULL, 10);
    dyne_perf_tx(&off, 1, 1, 1, 1, false);
    CHECK(off.n == 0 && off.dropped == 0, "no buffer: nothing recorded");
}

static void test_format(void)
{
    printf("[2] 1件の JSON\n");
    dyne_perf_rec_t mem[3];
    dyne_perf_t p;
    dyne_perf_init(&p, mem, 3);
    dyne_perf_tx(&p, 2, 7, 1795210811056000LL, 204, true);
    dyne_perf_rx(&p, 2, 7, 1795210811056000LL, 1795210811061234LL, 204);
    dyne_perf_step(&p, 7, 10, 20, 30);
    char s[256];
    dyne_perf_format(&mem[0], 3, s, sizeof(s));
    CHECK(strcmp(s, "{\"k\":\"tx\",\"node\":3,\"iface\":2,\"step\":7,\"t\":1795210811056000,\"bytes\":204,\"resend\":true}") == 0, "tx %s", s);
    dyne_perf_format(&mem[1], 3, s, sizeof(s));
    CHECK(strcmp(s, "{\"k\":\"rx\",\"node\":3,\"iface\":2,\"step\":7,\"t_sent\":1795210811056000,\"t\":1795210811061234,\"bytes\":204}") == 0, "rx %s", s);
    dyne_perf_format(&mem[2], 3, s, sizeof(s));
    CHECK(strcmp(s, "{\"k\":\"step\",\"node\":3,\"step\":7,\"t0\":10,\"t1\":20,\"t2\":30}") == 0, "step %s", s);
    CHECK(dyne_perf_format(&mem[0], 3, s, 20) == -1, "too small a buffer is reported");
}

static void test_unit_t(void)
{
    printf("[3] 配送単位の時刻印\n");
    uint8_t u[4 + 24 + 8];
    memset(u, 0, sizeof(u));
    u[0] = 24 + 8; /* 1つ目のメッセージの長さ */
    uint64_t t = 0x0102030405060708ULL;
    for (int k = 0; k < 8; k++) u[4 + 16 + k] = (uint8_t)(t >> (8 * k));
    CHECK((uint64_t)dyne_perf_unit_t_us(u, sizeof(u)) == t, "timestamp read");
    CHECK(dyne_perf_unit_t_us(u, 10) == 0, "short unit gives 0");
}

static void test_batch(void)
{
    printf("[4] まとめて送る形\n");
    dyne_perf_rec_t mem[50];
    dyne_perf_t p;
    dyne_perf_init(&p, mem, 50);
    for (uint32_t s = 0; s < 50; s++) dyne_perf_tx(&p, 1, s, 1000 + s, 100, false);
    char out[600];
    uint32_t i0 = 0, done, parts = 0;
    while (i0 < p.n) {
        int w = dyne_perf_batch(&p, 4, 50, i0, 1000, out, sizeof(out), &done);
        CHECK(w > 0 && done > 0, "batch at %u", (unsigned)i0);
        if (w <= 0 || done == 0) break;
        CHECK(out[w - 1] == '}' && strstr(out, "\"recs\":[") != NULL, "closed JSON");
        char want[32];
        snprintf(want, sizeof(want), ",\"n\":%u}", (unsigned)done);
        CHECK(strstr(out, want) != NULL, "count at the end");
        i0 += done;
        parts++;
    }
    CHECK(i0 == 50 && parts > 1, "all 50 in %u parts", (unsigned)parts);
    int w = dyne_perf_batch(&p, 4, 50, 0, 3, out, sizeof(out), &done);
    CHECK(w > 0 && done == 3, "max_n respected");
    CHECK(dyne_perf_batch(&p, 4, 50, 0, 3, out, 30, &done) == -1, "too small a buffer is reported");
}

static int emit(void)
{
    enum { STEPS = 100 };
    static dyne_perf_rec_t mem[2][4 * STEPS];
    dyne_perf_t p[2];
    for (int n = 0; n < 2; n++) dyne_perf_init(&p[n], mem[n], 4 * STEPS);
    const int64_t base = 1795210800000000LL;
    for (uint32_t s = 0; s < STEPS; s++) {
        int64_t t0[2], rx[2];
        for (int n = 0; n < 2; n++) {
            t0[n] = base + (int64_t)s * 100000 + n * 1000;
            dyne_perf_tx(&p[n], 1, s, t0[n], 204, false);
        }
        for (int n = 0; n < 2; n++) {
            int from = 1 - n;
            int64_t lat = 3000 + (int64_t)((s + (uint32_t)n) % 5) * 1000;
            rx[n] = t0[from] + lat;
            dyne_perf_rx(&p[n], 1, s, t0[from], rx[n], 204);
        }
        for (int n = 0; n < 2; n++) {
            int64_t t1 = rx[n] > t0[n] ? rx[n] : t0[n];
            dyne_perf_step(&p[n], s, t0[n], t1, t1 + 20000);
        }
    }
    static char out[2048];
    for (int n = 0; n < 2; n++) {
        uint32_t i0 = 0, done;
        while (i0 < p[n].n) {
            if (dyne_perf_batch(&p[n], n, STEPS, i0, 1000, out, sizeof(out), &done) < 0 || done == 0) return 1;
            printf("{\"t\":0.0,\"topic\":\"dyne/node/%d/perf\",\"payload\":%s}\n", n, out);
            i0 += done;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "--emit") == 0) return emit();
    test_push();
    test_format();
    test_unit_t();
    test_batch();
    printf("%s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
