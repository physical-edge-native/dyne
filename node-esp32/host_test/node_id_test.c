/*
 * ノード番号の決め方 (main/dyne_node_id_core.c) を PC 上で確かめる検査 (2026-11-15、GitHub issue #19)。
 *
 * 1. 決め方の順: menuconfig (0 以上) → NVS (範囲内) → Wi-Fi の MAC の末尾1バイト
 * 2. MAC の文字列の読み取り (":"・"-"・区切りなし、大文字小文字)
 * 3. ctl 命令 node_id の解釈 (報告・設定・消去・別の基板あて・失敗)
 * 4. issue #19 の完了の条件: MAC の番号が重なった2台に、同じ像のまま別の番号を与えられる
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -Inode-esp32\main ^
 *       node-esp32\host_test\node_id_test.c node-esp32\main\dyne_node_id_core.c -o %TEMP%\node_id_test.exe
 *   %TEMP%\node_id_test.exe   → 最終行が "PASS"
 * CI では scripts/ci-check.sh が cc (または gcc / clang) でビルドして実行する。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dyne_node_id_core.h"

static int g_fail;
#define CHECK(c, ...) do { if (!(c)) { g_fail++; printf("  NG: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const uint8_t MAC_A[6] = { 0x02, 0x00, 0x00, 0x00, 0x01, 0x25 }; /* 末尾 0x25 = 37 */
static const uint8_t MAC_B[6] = { 0x02, 0x00, 0x00, 0x00, 0x02, 0x25 }; /* 末尾が A と同じ */

static void test_select(void)
{
    printf("[1] 決め方の順\n");
    dyne_id_src_t s;
    int id;
    id = dyne_node_id_select(-1, false, 0, MAC_A, &s);
    CHECK(id == 37 && s == DYNE_ID_SRC_MAC, "no config, no nvs -> mac 37 (got %d %s)", id, dyne_id_src_name(s));
    id = dyne_node_id_select(-1, true, 5, MAC_A, &s);
    CHECK(id == 5 && s == DYNE_ID_SRC_NVS, "nvs 5 -> 5 nvs (got %d %s)", id, dyne_id_src_name(s));
    id = dyne_node_id_select(-1, true, 0, MAC_A, &s);
    CHECK(id == 0 && s == DYNE_ID_SRC_NVS, "nvs 0 is a valid id (got %d %s)", id, dyne_id_src_name(s));
    id = dyne_node_id_select(-1, true, 255, MAC_A, &s);
    CHECK(id == 255 && s == DYNE_ID_SRC_NVS, "nvs 255 (got %d)", id);
    id = dyne_node_id_select(7, true, 5, MAC_A, &s);
    CHECK(id == 7 && s == DYNE_ID_SRC_CONFIG, "config 7 overrides nvs (got %d %s)", id, dyne_id_src_name(s));
    id = dyne_node_id_select(0, false, 0, MAC_A, &s);
    CHECK(id == 0 && s == DYNE_ID_SRC_CONFIG, "config 0 (got %d %s)", id, dyne_id_src_name(s));
    /* 範囲外の NVS の値 (壊れた・古い版が書いた) は無視して MAC へ */
    int32_t bad[] = { -1, 256, 1000, -2147483647 - 1 };
    for (size_t k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
        id = dyne_node_id_select(-1, true, bad[k], MAC_A, &s);
        CHECK(id == 37 && s == DYNE_ID_SRC_MAC, "nvs %ld out of range -> mac (got %d %s)", (long)bad[k], id,
              dyne_id_src_name(s));
    }
    id = dyne_node_id_select(-1, false, 0, MAC_A, NULL);
    CHECK(id == 37, "src NULL allowed");
    CHECK(strcmp(dyne_id_src_name(DYNE_ID_SRC_CONFIG), "config") == 0 && strcmp(dyne_id_src_name(DYNE_ID_SRC_NVS), "nvs") == 0
              && strcmp(dyne_id_src_name(DYNE_ID_SRC_MAC), "mac") == 0, "src names");
    CHECK(dyne_node_id_valid(0) && dyne_node_id_valid(255) && !dyne_node_id_valid(256) && !dyne_node_id_valid(-1)
              && !dyne_node_id_valid(1.5) && !dyne_node_id_valid(NAN) && !dyne_node_id_valid(INFINITY),
          "valid range 0..255 integers");
}

static void test_mac_parse(void)
{
    printf("[2] MAC の文字列\n");
    uint8_t m[6];
    CHECK(dyne_mac_parse("02:00:00:00:01:25", m) && memcmp(m, MAC_A, 6) == 0, "colon");
    CHECK(dyne_mac_parse("02-00-00-00-01-25", m) && memcmp(m, MAC_A, 6) == 0, "dash");
    CHECK(dyne_mac_parse("020000000125", m) && memcmp(m, MAC_A, 6) == 0, "bare");
    const uint8_t up[6] = { 0xAB, 0xCD, 0xEF, 0x0A, 0xbc, 0xFF };
    CHECK(dyne_mac_parse("ab:CD:ef:0a:BC:ff", m) && memcmp(m, up, 6) == 0, "mixed case");
    const char *bad[] = { "", "02:00:00:00:01", "02:00:00:00:01:2", "02:00:00:00:01:256", "02:00-00:00:01:25",
                          "02.00.00.00.01.25", "0200000001zz", "02:00:00:00:01:25:", "02:00:00:00:01:2g" };
    for (size_t k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
        CHECK(!dyne_mac_parse(bad[k], m), "reject \"%s\"", bad[k]);
    }
    CHECK(!dyne_mac_parse(NULL, m), "reject NULL");
}

static void test_cmd(void)
{
    printf("[3] ctl 命令 node_id の解釈\n");
    int32_t out = -99;
    CHECK(dyne_node_id_cmd(NULL, false, 0, MAC_A, &out) == DYNE_IDCMD_QUERY, "no mac no id -> query");
    CHECK(dyne_node_id_cmd("02:00:00:00:01:25", false, 0, MAC_A, &out) == DYNE_IDCMD_QUERY, "my mac, no id -> query");
    CHECK(dyne_node_id_cmd("02:00:00:00:02:25", false, 0, MAC_A, &out) == DYNE_IDCMD_NOT_MINE, "other mac, no id -> not mine");
    out = -99;
    CHECK(dyne_node_id_cmd("02:00:00:00:01:25", true, 5, MAC_A, &out) == DYNE_IDCMD_SET && out == 5, "set 5");
    out = -99;
    CHECK(dyne_node_id_cmd("020000000125", true, 0, MAC_A, &out) == DYNE_IDCMD_SET && out == 0, "set 0 (bare mac)");
    CHECK(dyne_node_id_cmd("02:00:00:00:01:25", true, -1, MAC_A, &out) == DYNE_IDCMD_CLEAR, "id -1 -> clear");
    CHECK(dyne_node_id_cmd("02:00:00:00:02:25", true, 5, MAC_A, &out) == DYNE_IDCMD_NOT_MINE, "other mac -> not mine");
    CHECK(dyne_node_id_cmd("02:00:00:00:02:25", true, 999, MAC_A, &out) == DYNE_IDCMD_NOT_MINE,
          "other mac with bad id -> not mine (only the target answers)");
    CHECK(dyne_node_id_cmd(NULL, true, 5, MAC_A, &out) == DYNE_IDCMD_ERR_NO_MAC, "set without mac -> error");
    CHECK(dyne_node_id_cmd(NULL, true, -1, MAC_A, &out) == DYNE_IDCMD_ERR_NO_MAC, "clear without mac -> error");
    CHECK(dyne_node_id_cmd("02:00:00:00:01", true, 5, MAC_A, &out) == DYNE_IDCMD_ERR_BAD_MAC, "bad mac -> error");
    CHECK(dyne_node_id_cmd("nonsense", false, 0, MAC_A, &out) == DYNE_IDCMD_ERR_BAD_MAC, "bad mac query -> error");
    const double badid[] = { 256, -2, 1.5, NAN, INFINITY, 1e10 };
    for (size_t k = 0; k < sizeof(badid) / sizeof(badid[0]); k++) {
        CHECK(dyne_node_id_cmd("02:00:00:00:01:25", true, badid[k], MAC_A, &out) == DYNE_IDCMD_ERR_BAD_ID, "bad id %g",
              badid[k]);
    }
    CHECK(dyne_idcmd_err(DYNE_IDCMD_SET) == NULL && dyne_idcmd_err(DYNE_IDCMD_QUERY) == NULL
              && dyne_idcmd_err(DYNE_IDCMD_ERR_NO_MAC) != NULL && dyne_idcmd_err(DYNE_IDCMD_ERR_BAD_ID) != NULL,
          "error strings");
}

/* 基板1台の NVS を模擬して、命令 → 再起動 → 番号 の流れを確かめる */
typedef struct { const uint8_t *mac; bool has; int32_t val; int id; dyne_id_src_t src; } board_t;

static void boot(board_t *b)
{
    b->id = dyne_node_id_select(-1, b->has, b->val, b->mac, &b->src);
}

static void deliver(board_t *b, const char *mac_str, bool has_id, double id)
{
    int32_t v = -1;
    dyne_idcmd_t c = dyne_node_id_cmd(mac_str, has_id, id, b->mac, &v);
    if (c == DYNE_IDCMD_SET) { b->has = true; b->val = v; }
    if (c == DYNE_IDCMD_CLEAR) { b->has = false; }
}

static void test_two_boards(void)
{
    printf("[4] MAC の番号が重なった2台 (issue #19 の完了の条件)\n");
    board_t a = { MAC_A, false, 0, 0, 0 }, b = { MAC_B, false, 0, 0, 0 };
    boot(&a);
    boot(&b);
    CHECK(a.id == 37 && b.id == 37, "both start at 37 from mac");
    /* 両方が dyne/ctl/37 を購読しているので、同じ命令が両方に届く。mac で片方だけが従う */
    deliver(&a, "02:00:00:00:02:25", true, 38);
    deliver(&b, "02:00:00:00:02:25", true, 38);
    CHECK(a.id == 37 && b.id == 37, "ids do not change before reboot");
    boot(&a);
    boot(&b);
    CHECK(a.id == 37 && a.src == DYNE_ID_SRC_MAC, "A keeps 37 (mac) after reboot (got %d)", a.id);
    CHECK(b.id == 38 && b.src == DYNE_ID_SRC_NVS, "B gets 38 (nvs) after reboot (got %d)", b.id);
    /* mac の無い設定の命令は両方とも断る (両方が 38 になるのを防ぐ) */
    deliver(&a, NULL, true, 40);
    deliver(&b, NULL, true, 40);
    boot(&a);
    boot(&b);
    CHECK(a.id == 37 && b.id == 38, "set without mac changes nothing");
    /* 消すと MAC の番号に戻る */
    deliver(&b, "02:00:00:00:02:25", true, -1);
    boot(&b);
    CHECK(b.id == 37 && b.src == DYNE_ID_SRC_MAC, "clear -> back to mac 37 (got %d)", b.id);
}

int main(void)
{
    test_select();
    test_mac_parse();
    test_cmd();
    test_two_boards();
    if (g_fail) {
        printf("FAIL (%d)\n", g_fail);
        return 1;
    }
    printf("PASS\n");
    return 0;
}
