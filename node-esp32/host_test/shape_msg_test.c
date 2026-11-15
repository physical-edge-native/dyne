/*
 * 形の区画のメッセージ (main/dyne_shape_msg.c) を PC の参照実装 (scripts/dyne_shape_msg.py) と照合する検査
 * (2026-11-13)。この C のプログラムが決まった値の区画をいくつか組み立ててファイルに書き、
 * shape_msg_check.py が Python で読み (unpack)、同じ値を Python で組み立て直して (pack) バイト単位で比べる。
 *
 * 値: 区画 c の格子点 k の φ = (k·0.37 − 3.1)·(c+1)、d = k·0.01 − 0.05、w = (k mod 7)·0.5 を double で計算して
 * f32 へ丸める (Python 側も同じ式を double で計算して f32 へ丸めるので同じ値になる。C の側は掛け算と足し算を
 * 1命令 (FMA) にまとめないよう -ffp-contract=off でビルドする。まとめると d の値が1ビットずれる)。
 *
 * ビルドと実行の例 (リポジトリのルートで):
 *   .venv\Scripts\python.exe -m ziglang cc -std=c11 -Wall -Wextra -O2 -ffp-contract=off -Inode-esp32\main ^
 *       node-esp32\host_test\shape_msg_test.c node-esp32\main\dyne_shape_msg.c node-esp32\main\dyne_wire2.c ^
 *       -o %TEMP%\shape_msg_test.exe
 *   %TEMP%\shape_msg_test.exe %TEMP%\shape_tiles.bin
 *   .venv\Scripts\python.exe node-esp32\host_test\shape_msg_check.py %TEMP%\shape_tiles.bin   → 最終行が "PASS"
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dyne_shape_msg.h"
#include "dyne_wire2.h"

/* 区画の条件: (node, step, t_us, ix0, iy0, nx, ny, gnx, gny, h, x0, y0) */
typedef struct { uint16_t node; uint32_t step; uint64_t t_us; uint16_t ix0, iy0, nx, ny, gnx, gny; float h, x0, y0; } tc_t;

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s out.bin\n", argv[0]); return 2; }
    static const tc_t tc[] = {
        { 0, 0, 0, 0, 0, 1, 1, 0, 0, 0.0f, 0.0f, 0.0f },
        { 7, 123, 1731456789123456ull, 10, 20, 5, 4, 73, 55, 0.05f, 0.0f, 0.0f },
        { 13, 599, 1731456789999999ull, 109, 81, 36, 27, 145, 109, 0.025f, -0.125f, 0.25f }, /* 形の縮小版の区画 */
        { 31, 4000000000u, 0xFFFFFFFFFFFFull, 0, 0, 18, 27, 0, 0, 0.025f, 0.0f, 0.0f },     /* 約 5.9 kB */
    };
    FILE *f = fopen(argv[1], "wb");
    if (!f) { perror("fopen"); return 2; }
    for (size_t c = 0; c < sizeof(tc) / sizeof(tc[0]); c++) {
        const tc_t *t = &tc[c];
        size_t n = (size_t)t->nx * t->ny, len = dyne_shape_msg_len(t->nx, t->ny);
        uint8_t *buf = malloc(len);
        for (size_t k = 0; k < n; k++) {
            dyne_shape_put(buf, k, (float)(((double)k * 0.37 - 3.1) * (double)(c + 1)));
            dyne_shape_put(buf, n + k, (float)((double)k * 0.01 - 0.05));
            dyne_shape_put(buf, 2 * n + k, (float)((double)(k % 7) * 0.5));
        }
        dyne_shape_hdr_t h = { .node = t->node, .step = t->step, .t_us = t->t_us, .ix0 = t->ix0, .iy0 = t->iy0,
                               .nx = t->nx, .ny = t->ny, .gnx = t->gnx, .gny = t->gny, .h = t->h, .x0 = t->x0,
                               .y0 = t->y0 };
        size_t got = dyne_shape_finish(buf, &h, dyne_env_crc32);
        if (got != len) { fprintf(stderr, "len mismatch\n"); return 1; }
        uint32_t l32 = (uint32_t)len;
        fwrite(&l32, 4, 1, f); /* ファイル内の区切り (PC はリトルエンディアン) */
        fwrite(buf, 1, len, f);
        printf("tile %zu: %zu bytes\n", c, len);
        free(buf);
    }
    fclose(f);
    return 0;
}
