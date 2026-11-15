/*
 * 運用の仕組み (2026-11-11): OTA (無線経由の書き換え)、計算部品のパーティション、
 * 起動後の自己確認と元に戻す処理、再起動の理由と稼働時間の報告。
 * 説明: docs/2026-11-11-timesync-ota.md
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"
#include "dyne_actor_slot.h"

/* 起動直後 (Wi-Fi より前) に1回: NVS の初期化 (パーティションテーブルの変更後は消去して作り直す)、
 * 再起動の理由・起動回数・前回の稼働時間の記録 */
void dyne_ops_boot_early(void);

/* ==== 計算部品の候補 (パーティション) ==== */
typedef struct {
    int slot;          /* 0 / 1 */
    bool aot;          /* true = 事前コンパイル済み (.aot) */
    uint32_t seq;
    char name[DYNE_ACT_NAME_LEN];
} dyne_actor_cand_t;

/* 試す順の候補 (新しい枠の .aot → .wasm → 古い枠の …)。パーティションが無い・空なら 0 個 */
int dyne_actor_candidates(dyne_actor_cand_t *out, int max);
/* 候補を読み出し、SHA-256 と長さを確かめて malloc した領域で返す。失敗は NULL */
uint8_t *dyne_actor_read(const dyne_actor_cand_t *c, uint32_t *len);
/* 実際に使った候補を記録する (NULL = ファームに埋め込んだ計算部品) */
void dyne_actor_set_loaded(const dyne_actor_cand_t *c);

/* Wi-Fi より前に1回: 起動後の自己確認のタスクを始める。
 * OTA 直後の起動 (確認待ち) なら、期限までに Wi-Fi・MQTT・自己検査がそろわないと
 * 前のファームに戻して再起動する。selftest_ok = 外箱の自己検査と計算部品の積み込みの結果 */
void dyne_ops_start_selfcheck(bool selftest_ok);

/* MQTT 開始後に1回: 稼働時間などの定期報告 (dyne/node/<id>/health) */
void dyne_ops_start_health(void);

/* ctl 命令の処理。扱った命令なら true */
bool dyne_ops_on_ctl(const char *cmd, const cJSON *root);

/* online の status に足す項目 (JSON の断片、先頭カンマなし) */
int dyne_ops_online_fields(char *buf, int len);
/* 起動からの秒数 */
uint32_t dyne_ops_uptime_s(void);
