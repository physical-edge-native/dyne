/*
 * ワイヤ形式v2 の外箱 (UDPの1データグラム) の C 実装 (2026-11-08)
 *
 * 単一定義は Rust 側 crates/dyne-wire/src/envelope.rs (レイアウト表もそちら)。
 * 本ファイルはファームが同じバイト列を作る・検査するための写しで、食い違いは
 * 照合用バイト列 dyne_wire2_golden.h を Rust のテストと C の起動時自己検査の
 * 両方で確かめて検出する。説明: docs/2026-11-08-firmware-wire-v2.md
 *
 * 外箱 = 前置部32バイト (識別子 "DYN2"、版1、送信元、通番、界面ID、中身の長さ、
 * ステップ) + 中身 (1つの界面・1つのステップぶんの枠つき v2 メッセージ列) +
 * CRC32 4バイト。1データグラムは 1472 バイトまで。
 *
 * ESP-IDF に依存しない (PC の C コンパイラでもそのままビルドできる)。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define DYNE_ENV_HDR_LEN      32
#define DYNE_ENV_CRC_LEN      4
#define DYNE_ENV_MAX_DATAGRAM 1472
#define DYNE_ENV_MAX_PAYLOAD  (DYNE_ENV_MAX_DATAGRAM - DYNE_ENV_HDR_LEN - DYNE_ENV_CRC_LEN)
#define DYNE_ENV_VERSION      1

/* ワイヤ形式v2 の定数 (crates/dyne-wire/src/lib.rs と同じ値) */
#define DYNE_V2_VERSION       2
#define DYNE_V2_HDR_LEN       24
#define DYNE_V2_KIND_IFACE_SCALAR 0
#define DYNE_V2_KIND_IFACE_BAND   1
#define DYNE_V2_KIND_POSITION     2
#define DYNE_V2_KIND_HEARSAY      3
#define DYNE_V2_KIND_RANGE_FRAME  4
#define DYNE_V2_KIND_RAYS         5

/* 誤りコード (Rust の EnvError と対応) */
#define DYNE_ENV_OK          0
#define DYNE_ENV_E_TRUNCATED (-1)
#define DYNE_ENV_E_OVERSIZE  (-2)
#define DYNE_ENV_E_FULL      (-3)
#define DYNE_ENV_E_MAGIC     (-4)
#define DYNE_ENV_E_VERSION   (-5)
#define DYNE_ENV_E_CRC       (-6)
#define DYNE_ENV_E_UNIT      (-7)

typedef struct {
    uint32_t sender; /* 送信元ノードID */
    uint32_t seq;    /* 通番 */
    uint32_t iface;  /* 界面ID */
    uint64_t step;   /* ステップ */
} dyne_env_hdr_t;

/* IEEE CRC32 (既存の dyne_crc32 と同じ式。本ファイル単独でビルドできるよう持つ) */
uint32_t dyne_env_crc32(const uint8_t *d, size_t len);

/* v2 メッセージ1通 (raw, len バイト) を検査し、種別・src・step を返す。
 * len がちょうど1通の長さであること。戻り値 0 / DYNE_ENV_E_UNIT */
int dyne_v2_peek(const uint8_t *raw, size_t len, uint8_t *kind, uint32_t *src, uint64_t *step);

/* 界面ID で運ぶ種別か (Rust の routed_by_iface と同じ) */
bool dyne_v2_routed_by_iface(uint8_t kind);

/* 中身 (枠つき v2 メッセージ列、終端印なし) が iface/step の配送単位として正しいか。
 * 戻り値: 通数 (1以上) / DYNE_ENV_E_UNIT */
int dyne_env_check_unit(const uint8_t *payload, size_t len, uint32_t iface, uint64_t step);

/* 外箱に包んで out へ書く。戻り値: データグラムのバイト数 / 負の誤りコード
 * (誤りのとき out には書かない) */
int dyne_env_encode(uint8_t *out, size_t cap, const dyne_env_hdr_t *h,
                    const uint8_t *payload, size_t plen);

/* 受信したデータグラムを検査する。成功時は *payload が dgram 内の中身を指す。
 * 戻り値: 0 / 負の誤りコード。検査の順は Rust 側と同じ */
int dyne_env_decode(const uint8_t *dgram, size_t len, dyne_env_hdr_t *h,
                    const uint8_t **payload, size_t *plen);

/* 照合用バイト列との自己検査 (符号化の一致、往復、1ビット破損の検出)。
 * 戻り値: true = 合格 */
bool dyne_env_selftest(void);
