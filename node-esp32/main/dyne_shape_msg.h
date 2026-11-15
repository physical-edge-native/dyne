/*
 * 形の区画のメッセージ (版1、"DSF1") の C 実装 (2026-11-13)
 *
 * 定義の正本は docs/2026-11-13-pc-pipeline-rl.md §5.2、PC 側の参照実装は scripts/dyne_shape_msg.py。
 * MQTT のトピック dyne/node/<node.idx>/shape、QoS 0、バイナリ (リトルエンディアン):
 *   52 バイトの見出し (識別・版・符号化・見出しの長さ・ノード番号・ステップ・時刻印・区画・全体の格子・
 *   刻みと原点・本体の CRC-32) + 本体 φ[nx·ny]、d[nx·ny]、w[nx·ny] (f32、格子点 k = j·nx + i)。
 * 両者のバイト一致は node-esp32/host_test/shape_msg_test.c + shape_msg_check.py で確かめる。
 *
 * ESP-IDF に依存しない (PC の C コンパイラでもビルドできる)。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define DYNE_SHAPE_HDR_LEN 52
#define DYNE_SHAPE_VERSION 1
#define DYNE_SHAPE_ENC_F32 1

typedef struct {
    uint16_t node;          /* 設定注入の node.idx */
    uint32_t step;          /* この値を計算し終えたステップ */
    uint64_t t_us;          /* 同期した時刻 [µs] (同期前は 0) */
    uint16_t ix0, iy0, nx, ny;
    uint16_t gnx, gny;      /* 全体の格子の点の数 (分からなければ 0) */
    float h, x0, y0;        /* 刻み・全体の格子の原点 [m] (分からなければ 0) */
} dyne_shape_hdr_t;

/* メッセージ全体の長さ */
size_t dyne_shape_msg_len(uint16_t nx, uint16_t ny);

/* 本体の i 番目の f32 (0..3·nx·ny-1: φ, d, w の順) を書く。buf はメッセージの先頭 */
void dyne_shape_put(uint8_t *buf, size_t i, float v);

/* 本体を書き終えた buf に見出し (本体の CRC-32 を含む) を書く。
 * crc32 は IEEE 802.3 の CRC を計算する関数 (dyne_env_crc32 を渡す)。戻り値: メッセージの長さ */
size_t dyne_shape_finish(uint8_t *buf, const dyne_shape_hdr_t *h,
                         uint32_t (*crc32)(const uint8_t *, size_t));
