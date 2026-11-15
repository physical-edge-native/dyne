/*
 * 計算部品の保存区画の形式と選び方 (main/dyne_actor_slot.c) を PC 上で確かめる検査 (2026-11-11)。
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -Inode-esp32\main ^
 *       node-esp32\host_test\actor_slot_test.c node-esp32\main\dyne_actor_slot.c -o %TEMP%\actor_slot_test.exe
 *   %TEMP%\actor_slot_test.exe      → 最終行が "PASS"
 *
 * 引数に 128 バイトの見出しを16進で渡すと、それを解釈して表示する
 * (PC 側の scripts/dyne_ota.py が同じ形式を作れることの照合用)。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dyne_actor_slot.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  NG: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void sim_slot_selection(void)
{
    printf("[3] 書き込みと選び方の模擬\n");
    bool valid[2] = { false, false };
    uint32_t seq[2] = { 0, 0 };
    int order[2];
    CHECK(dyne_act_order(valid, seq, order) == 0, "empty order");
    CHECK(dyne_act_write_slot(valid, seq) == 0, "first write slot");
    /* 1本目 → 枠0 */
    int w = dyne_act_write_slot(valid, seq);
    seq[w] = dyne_act_next_seq(valid, seq); valid[w] = true;
    CHECK(w == 0 && seq[0] == 1, "first");
    /* 2本目 → 枠1 (1本目を残す) */
    w = dyne_act_write_slot(valid, seq);
    seq[w] = dyne_act_next_seq(valid, seq); valid[w] = true;
    CHECK(w == 1 && seq[1] == 2, "second");
    CHECK(dyne_act_order(valid, seq, order) == 2 && order[0] == 1 && order[1] == 0, "order newest first");
    /* 3本目 → 枠0 (2本目を残す) */
    w = dyne_act_write_slot(valid, seq);
    CHECK(w == 0, "third goes to slot0");
    /* 書き込み中の電源断: 見出しは消えたまま → 枠1 だけが有効 */
    valid[0] = false;
    CHECK(dyne_act_order(valid, seq, order) == 1 && order[0] == 1, "torn write falls back to slot1");
    /* 書き直し → 枠0 に seq=3 */
    w = dyne_act_write_slot(valid, seq);
    seq[w] = dyne_act_next_seq(valid, seq); valid[w] = true;
    CHECK(w == 0 && seq[0] == 3, "rewrite seq=%u", seq[0]);
    /* 前の計算部品に戻す命令 = 新しい方の見出しを消す */
    valid[0] = false;
    CHECK(dyne_act_order(valid, seq, order) == 1 && order[0] == 1, "rollback");

    /* 起動の失敗が続く新しい計算部品を飛ばす */
    CHECK(!dyne_act_should_skip(5, 4, 1, 2), "try1");
    CHECK(!dyne_act_should_skip(5, 4, 2, 2), "try2");
    CHECK(dyne_act_should_skip(5, 4, 3, 2), "try3 skip");
    CHECK(!dyne_act_should_skip(5, 5, 99, 2), "already ok never skipped");
}

int main(int argc, char **argv)
{
    if (argc >= 2) {
        const char *hx = argv[1];
        uint8_t b[DYNE_ACT_HDR_LEN];
        if (strlen(hx) != 2 * DYNE_ACT_HDR_LEN) { printf("need %d hex chars\n", 2 * DYNE_ACT_HDR_LEN); return 2; }
        for (int i = 0; i < DYNE_ACT_HDR_LEN; i++) { unsigned v; sscanf(hx + 2 * i, "%2x", &v); b[i] = (uint8_t)v; }
        dyne_act_hdr_t h;
        int rc = dyne_act_hdr_decode(b, 1u << 20, &h);
        printf("decode=%d seq=%u wasm=%u aot=%u name=%s wasm_sha0=%02x aot_off=%u\n", rc, h.seq,
               h.wasm_size, h.aot_size, h.name, h.wasm_sha[0], dyne_act_aot_off(&h));
        return rc == 0 ? 0 : 1;
    }

    printf("[1] 見出しの符号化と検査\n");
    dyne_act_hdr_t h = { 0 };
    h.seq = 7; h.wasm_size = 3616; h.aot_size = 20000;
    for (int i = 0; i < 32; i++) { h.wasm_sha[i] = (uint8_t)i; h.aot_sha[i] = (uint8_t)(255 - i); }
    strcpy(h.name, "heat1d_actor_small");
    uint8_t b[DYNE_ACT_HDR_LEN];
    dyne_act_hdr_encode(b, &h);
    dyne_act_hdr_t g;
    CHECK(dyne_act_hdr_decode(b, 352 * 1024, &g) == 0, "decode ok");
    CHECK(g.seq == 7 && g.wasm_size == 3616 && g.aot_size == 20000 && !strcmp(g.name, h.name)
          && !memcmp(g.wasm_sha, h.wasm_sha, 32) && !memcmp(g.aot_sha, h.aot_sha, 32), "fields");
    CHECK(dyne_act_wasm_off(&g) == 4096 && dyne_act_aot_off(&g) == 8192, "offsets");
    CHECK(dyne_act_span(3616, 20000) == 4096 + 4096 + 20480, "span");
    printf("    見出し (seq=7) の16進: ");
    for (int i = 0; i < DYNE_ACT_HDR_LEN; i++) printf("%02x", b[i]);
    printf("\n");

    printf("[2] 壊れた見出しを受理しない\n");
    uint8_t e[DYNE_ACT_HDR_LEN];
    memset(e, 0xFF, sizeof(e)); /* 消去したままのフラッシュ */
    CHECK(dyne_act_hdr_decode(e, 352 * 1024, &g) == -1, "erased = empty");
    int miss = 0;
    for (int bit = 0; bit < DYNE_ACT_HDR_LEN * 8; bit++) {
        memcpy(e, b, sizeof(e));
        e[bit / 8] ^= (uint8_t)(1u << (bit % 8));
        if (dyne_act_hdr_decode(e, 352 * 1024, &g) == 0) miss++;
    }
    CHECK(miss == 0, "bit flips accepted=%d", miss);
    CHECK(dyne_act_hdr_decode(b, 16 * 1024, &g) == -4, "too large for slot");

    sim_slot_selection();
    printf("%s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
