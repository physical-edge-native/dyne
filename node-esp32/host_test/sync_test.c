/*
 * ワイヤ形式v2 の経路のステップ同期の受信の判定 (main/dyne_sync_core.c) を PC 上で確かめる検査
 * (2026-11-15、GitHub issue #29)。
 *
 * 1. 受信の扱いの表 (受け取る・取り置く・過去分に応える・同じステップの重複に応える・捨てる)
 * 2. 同じステップの重複への応答の間隔 (界面ごとに再送の最小間隔 20 ms 以上。ステップの始めで消える)
 * 3. 輪になった4台 (2×2 の分割) の行き詰まり: docs/2026-11-15-sim2d-loss-jitter.md の表と同じ状態から、
 *    修正前の規則 (同じステップの重複を捨てる) では何回再送しても誰も進まず、修正後の規則では1回の再送で全員が揃う
 * 4. 止まっていることの報告の時期
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -Inode-esp32\main ^
 *       node-esp32\host_test\sync_test.c node-esp32\main\dyne_sync_core.c -o %TEMP%\sync_test.exe
 *   %TEMP%\sync_test.exe   → 最終行が "PASS"
 * CI では scripts/ci-check.sh の [6] が cc (または gcc / clang) でビルドして実行する。
 */
#include <stdio.h>
#include <string.h>
#include "dyne_sync_core.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  NG: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define GAP 20000 /* node_v2.c の V2_RESEND_MIN_US、dyne-sim2d の resend_min_us の既定と同じ */

static void test_table(void)
{
    printf("[1] 受信の扱いの表\n");
    dyne_sync_echo_t e;
    dyne_sync_echo_reset(&e);
    CHECK(dyne_sync_classify(5, 5, false, &e, 0, GAP) == DYNE_SYNC_ACCEPT, "same step, not yet got -> accept");
    CHECK(dyne_sync_classify(5, 6, false, &e, 0, GAP) == DYNE_SYNC_HOLD, "future -> hold");
    CHECK(dyne_sync_classify(5, 6, true, &e, 0, GAP) == DYNE_SYNC_HOLD, "future (got) -> hold");
    CHECK(dyne_sync_classify(5, 4, false, &e, 0, GAP) == DYNE_SYNC_REPLY_PAST, "past -> reply past");
    CHECK(dyne_sync_classify(5, 4, true, &e, 0, GAP) == DYNE_SYNC_REPLY_PAST, "past (got) -> reply past");
    CHECK(dyne_sync_classify(5, 0, true, &e, 0, GAP) == DYNE_SYNC_REPLY_PAST, "far past -> reply past (slot check is the caller's)");
    CHECK(dyne_sync_classify(5, 5, true, &e, 0, GAP) == DYNE_SYNC_REPLY_SAME, "same step duplicate -> reply same");
    CHECK(dyne_sync_classify(0, 0, true, &e, 0, GAP) == DYNE_SYNC_REPLY_SAME, "step 0 duplicate -> reply same");
    CHECK(dyne_sync_classify(UINT64_MAX, UINT64_MAX - 1, true, &e, 0, GAP) == DYNE_SYNC_REPLY_PAST, "large steps");
}

static void test_gap(void)
{
    printf("[2] 同じステップの重複への応答の間隔\n");
    dyne_sync_echo_t e;
    dyne_sync_echo_reset(&e);
    int64_t t = 1000000;
    CHECK(dyne_sync_classify(7, 7, true, &e, t, GAP) == DYNE_SYNC_REPLY_SAME, "first duplicate -> reply");
    /* classify は状態を変えない: mark するまでは何度でも REPLY_SAME */
    CHECK(dyne_sync_classify(7, 7, true, &e, t, GAP) == DYNE_SYNC_REPLY_SAME, "classify is pure");
    dyne_sync_echo_mark(&e, t);
    CHECK(dyne_sync_classify(7, 7, true, &e, t, GAP) == DYNE_SYNC_DROP, "same time -> drop");
    CHECK(dyne_sync_classify(7, 7, true, &e, t + GAP - 1, GAP) == DYNE_SYNC_DROP, "gap - 1 us -> drop");
    CHECK(dyne_sync_classify(7, 7, true, &e, t + GAP, GAP) == DYNE_SYNC_REPLY_SAME, "gap -> reply");
    CHECK(dyne_sync_classify(7, 7, false, &e, t, GAP) == DYNE_SYNC_ACCEPT, "rate limit never blocks a first copy");
    CHECK(dyne_sync_classify(7, 6, true, &e, t, GAP) == DYNE_SYNC_REPLY_PAST, "rate limit does not apply to past replies");
    /* 時刻 0 の応答も記録として扱う (echoed の旗で区別する) */
    dyne_sync_echo_t z;
    dyne_sync_echo_reset(&z);
    dyne_sync_echo_mark(&z, 0);
    CHECK(dyne_sync_classify(1, 1, true, &z, 5, GAP) == DYNE_SYNC_DROP, "echo at t=0 is remembered");
    /* ステップの始めで消える */
    dyne_sync_echo_reset(&e);
    CHECK(dyne_sync_classify(8, 8, true, &e, t + 1, GAP) == DYNE_SYNC_REPLY_SAME, "reset at step start -> reply");
}

/* ---- 3. 輪になった4台 ----
 * dyne-sim2d の 2×2 の分割と同じ界面の番号: ノード0 = [0, 2]、ノード1 = [0, 3]、ノード2 = [1, 2]、ノード3 = [1, 3]。
 * 各ノードはステップ S にいて、自分の値はすでに両隣へ届いている。足りないのは1つだけ
 * (ノード0←2、ノード1←0、ノード2←3、ノード3←1。docs/2026-11-15-sim2d-loss-jitter.md の表)。
 * 1回り = 返事待ちの上限 (100 ms) ごと: 各ノードは足りない界面へ自分の控えを再送し、届いたものを順に処理する。 */
#define NN 4
#define S 60
static const int IFS[NN][2] = { { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 } };
static const int PEER[NN][2] = { { 1, 2 }, { 0, 3 }, { 3, 0 }, { 2, 1 } }; /* 界面の向こうのノード */

typedef struct { int to, iface; uint64_t step; } msg_t;

/* old_rule: 修正前 (同じステップの重複は捨てる)。戻り値: 全員が揃った回り (揃わなければ -1) */
static int ring_run(bool old_rule, int rounds, uint64_t *n_echo)
{
    bool got[NN][2] = { { true, false }, { false, true }, { false, true }, { true, false } };
    uint64_t step[NN];
    dyne_sync_echo_t ec[NN][2];
    for (int n = 0; n < NN; n++) {
        step[n] = S;
        for (int k = 0; k < 2; k++) dyne_sync_echo_reset(&ec[n][k]);
    }
    *n_echo = 0;
    for (int r = 0; r < rounds; r++) {
        int64_t now = (int64_t)r * 100000;
        msg_t q[256];
        int qh = 0, qt = 0;
        for (int n = 0; n < NN; n++) /* 足りない界面へ再送 (待っているノードだけ) */
            for (int k = 0; k < 2; k++)
                if (step[n] == S && !got[n][k]) q[qt++] = (msg_t){ PEER[n][k], IFS[n][k], S };
        while (qh < qt && qt < 250) {
            msg_t m = q[qh++];
            int n = m.to, k = IFS[n][0] == m.iface ? 0 : 1;
            if (step[n] != S) {
                /* 揃って先へ進んだノード: 過去分として控えを返す (判定は表のとおり REPLY_PAST)。
                 * 揃ったノードどうしの過去分の往復は、ここでは追わない (2ステップの控えの範囲で止まる) */
                CHECK(dyne_sync_classify(step[n], m.step, true, &ec[n][k], now, GAP) == DYNE_SYNC_REPLY_PAST, "moved on -> past");
                if (step[PEER[n][k]] == S) q[qt++] = (msg_t){ PEER[n][k], m.iface, S };
                continue;
            }
            dyne_sync_rx_t a = dyne_sync_classify(step[n], m.step, got[n][k], &ec[n][k], now, GAP);
            if (old_rule && a == DYNE_SYNC_REPLY_SAME) a = DYNE_SYNC_DROP;
            if (a == DYNE_SYNC_ACCEPT) {
                got[n][k] = true;
                if (got[n][0] && got[n][1]) step[n] = S + 1; /* 揃った: 計算して次へ (次のステップの交換はここでは追わない) */
            } else if (a == DYNE_SYNC_REPLY_SAME) {
                dyne_sync_echo_mark(&ec[n][k], now);
                (*n_echo)++;
                q[qt++] = (msg_t){ PEER[n][k], m.iface, S };
            }
        }
        bool all = true;
        for (int n = 0; n < NN; n++) all &= step[n] == S + 1;
        if (all) return r + 1;
    }
    return -1;
}

static void test_ring(void)
{
    printf("[3] 輪になった4台の行き詰まり (2x2)\n");
    uint64_t echo = 0;
    int r_old = ring_run(true, 1000, &echo);
    CHECK(r_old == -1, "old rule: still stuck after 1000 resend rounds (got %d)", r_old);
    CHECK(echo == 0, "old rule: no echo");
    int r_new = ring_run(false, 1000, &echo);
    CHECK(r_new == 1, "new rule: all four nodes complete the step in the first resend round (got %d)", r_new);
    CHECK(echo == 4, "new rule: one echo per node (got %llu)", (unsigned long long)echo);
    printf("  old rule: %s, new rule: done in %d round(s), %llu echo(es)\n", r_old < 0 ? "stuck" : "done", r_new,
           (unsigned long long)echo);
}

static void test_stall(void)
{
    printf("[4] 止まっていることの報告の時期\n");
    int64_t w = 5000000, st = 10000000;
    CHECK(!dyne_sync_stall_due(w, w, st, 0), "just started");
    CHECK(!dyne_sync_stall_due(w, w + st - 1, st, 0), "stall - 1 us");
    CHECK(dyne_sync_stall_due(w, w + st, st, 0), "stall -> first report");
    CHECK(!dyne_sync_stall_due(w, w + st, st, 1), "after the first report");
    CHECK(!dyne_sync_stall_due(w, w + 2 * st - 1, st, 1), "2 stall - 1 us");
    CHECK(dyne_sync_stall_due(w, w + 2 * st, st, 1), "2 stall -> second report");
    CHECK(!dyne_sync_stall_due(w, w + 1000 * st, 0, 0), "0 = off");
    CHECK(!dyne_sync_stall_due(w, w + 1000 * st, -1, 0), "negative = off");
}

int main(void)
{
    test_table();
    test_gap();
    test_ring();
    test_stall();
    printf("%s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
