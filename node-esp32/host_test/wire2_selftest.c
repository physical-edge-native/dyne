/*
 * 外箱の C 実装 (main/dyne_wire2.c) を PC 上で確かめる小さな検査 (2026-11-08)。
 * ファームの起動時自己検査 dyne_env_selftest() と同じものを PC で実行し、
 * 照合用バイト列 (main/dyne_wire2_golden.h、Rust のテストも同じ配列を検査する) を
 * 16進で表示する。
 *
 * ビルドと実行の例 (リポジトリのルートで。C コンパイラは .venv の ziglang を使う):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -Inode-esp32\main ^
 *       node-esp32\host_test\wire2_selftest.c node-esp32\main\dyne_wire2.c -o %TEMP%\wire2_selftest.exe
 *   %TEMP%\wire2_selftest.exe      → 最終行が "PASS" であること
 */
#include <stdio.h>
#include "dyne_wire2.h"
#include "dyne_wire2_golden.h"

int main(void)
{
    printf("golden (%u bytes):", (unsigned)sizeof(DYNE_ENV_GOLDEN_BYTES));
    for (size_t i = 0; i < sizeof(DYNE_ENV_GOLDEN_BYTES); i++)
        printf("%s%02X", i % 16 ? " " : "\n  ", DYNE_ENV_GOLDEN_BYTES[i]);
    printf("\ncrc32(\"123456789\") = 0x%08X (expect 0xCBF43926)\n",
           (unsigned)dyne_env_crc32((const uint8_t *)"123456789", 9));
    int ok = dyne_env_selftest() && dyne_env_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u;
    printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
