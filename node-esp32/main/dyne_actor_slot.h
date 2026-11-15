/*
 * 計算部品 (WASM アクター) の保存区画の形式と選び方 (ESP-IDF に依存しない) (2026-11-11)
 *
 * 説明: docs/2026-11-11-timesync-ota.md
 *
 * 区画 "actors" (パーティションテーブルの data 種別、副種別 0x40) を2つの枠 (枠0・枠1) に等分する。
 * 1つの枠の並び:
 *   +0      見出し 128 バイト (下記。4KB の消去単位を1つ占める)
 *   +4096   .wasm 本体
 *   +4096+切り上げ(wasm の長さ, 4096)   .aot 本体 (事前コンパイル済み。無くてもよい)
 *
 * 書き込みの順: 見出しの消去 → 本体の書き込み → SHA-256 と長さの照合 → 見出しの書き込み (最後)。
 * 途中で電源が落ちても、見出しが無い枠は「空」として扱われ、もう一方の枠が使われる。
 *
 * 選び方: 有効な見出しを持つ枠のうち、通し番号 (seq) が大きい方を先に試す。
 * 積み込みに失敗したらもう一方、それも駄目ならファームに埋め込んだ計算部品を使う。
 * 新しい計算部品は常に「いま優先されていない方の枠」に書く (直前の計算部品を残す)。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define DYNE_ACT_MAGIC     0x43415944u /* "DYAC" (リトルエンディアンで並べた4文字) */
#define DYNE_ACT_VERSION   1
#define DYNE_ACT_HDR_LEN   128
#define DYNE_ACT_SECTOR    4096
#define DYNE_ACT_NAME_LEN  24

typedef struct {
    uint32_t seq;                    /* 通し番号 (大きいほど新しい) */
    uint32_t wasm_size;              /* 0 = .wasm なし */
    uint8_t  wasm_sha[32];
    uint32_t aot_size;               /* 0 = .aot なし */
    uint8_t  aot_sha[32];
    char     name[DYNE_ACT_NAME_LEN];/* 計算部品の名前 (表示用、終端0込み) */
} dyne_act_hdr_t;

/* 本体の置き場所 (枠の先頭からのバイト位置) */
uint32_t dyne_act_wasm_off(const dyne_act_hdr_t *h);
uint32_t dyne_act_aot_off(const dyne_act_hdr_t *h);
/* 枠の中で使う長さ (消去する範囲。4096 の倍数) */
uint32_t dyne_act_span(uint32_t wasm_size, uint32_t aot_size);

/* 見出しの符号化 (CRC32 つき) と検査。decode の戻り値: 0 = 有効 /
 * -1 識別子なし (空) / -2 版 / -3 CRC / -4 長さが枠を超える / -5 本体なし */
void dyne_act_hdr_encode(uint8_t out[DYNE_ACT_HDR_LEN], const dyne_act_hdr_t *h);
int dyne_act_hdr_decode(const uint8_t in[DYNE_ACT_HDR_LEN], uint32_t slot_size, dyne_act_hdr_t *h);

/* 試す順。valid[i] = 枠i の見出しが有効か、seq[i] = その通し番号。
 * order に試す枠の番号を新しい順に入れ、個数を返す */
int dyne_act_order(const bool valid[2], const uint32_t seq[2], int order[2]);
/* 次に書く枠 (いま優先されていない方。両方空なら枠0) */
int dyne_act_write_slot(const bool valid[2], const uint32_t seq[2]);
/* 次に付ける通し番号 */
uint32_t dyne_act_next_seq(const bool valid[2], const uint32_t seq[2]);

/* 起動の失敗が続く新しい計算部品を飛ばすか。
 * newest_seq = 最も新しい枠の通し番号、ok_seq = 起動後の自己確認に合格した最後の通し番号、
 * tries = その newest_seq で起動を試みた回数 (今回を含む) */
bool dyne_act_should_skip(uint32_t newest_seq, uint32_t ok_seq, uint32_t tries, uint32_t max_tries);

/* IEEE CRC32 (dyne_crc32 と同じ式。本ファイル単独でビルドできるよう持つ) */
uint32_t dyne_act_crc32(const uint8_t *d, size_t len);
