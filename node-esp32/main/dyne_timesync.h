/*
 * 時刻同期 (2026-11-11)。説明: docs/2026-11-11-timesync-ota.md
 *
 *  - ESP-IDF 標準の SNTP で、システムの時計 (gettimeofday) を PC に合わせる
 *  - 独自に PC の時刻サーバと往復の時刻印を交わし、ずれ・ドリフト・往復時間を推定する。
 *    「同期した時刻」= esp_timer (起動からの µs) + 推定したずれ。測距・センサの時刻印と
 *    試験モードのピン切り替えはこちらを使う (SNTP は往復時間を補正しないため。§設計)
 *  - 推定の状態を dyne/node/<id>/tsync に定期的に報告する
 *  - 試験モード: 同期した時刻の整数秒ちょうどに GPIO を切り替える (ロジックアナライザで
 *    ノード間のずれを測る)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"

/* Wi-Fi 接続後に1回呼ぶ (SNTP と往復の交換のタスクを始める) */
void dyne_timesync_start(void);

/* ctl 命令 {"cmd":"timesync", ...} の処理 (設定の変更・試験モードの開始/停止) */
void dyne_timesync_on_ctl(const cJSON *root);

/* 同期した時刻が使えるか */
bool dyne_time_synced(void);
/* いまの同期した時刻 (1970-01-01 からの µs)。同期前は 0 */
int64_t dyne_time_now_us(void);
/* esp_timer の時刻 (起動からの µs) を同期した時刻へ変換。同期前は 0 */
int64_t dyne_time_at_local(int64_t local_us);

/* 状態の要約を JSON の断片 (先頭カンマなし) で書く (health 報告に使う) */
int dyne_timesync_summary(char *buf, int len);
