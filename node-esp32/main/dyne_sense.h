/*
 * 測距 (VL53L5CX)・サーボの首振り (PAN4H)・IMU (LSM6DSOX) のノードへの組み込み (2026-11-13)
 *
 * 説明: docs/2026-11-13-firmware-nodes.md。計算部分 (PC で検査できる部分) は dyne_sense_core.c。
 *
 * どのノードも同じファーム。センサは起動時に I2C で探し、見つかったものだけ動かす
 * (menuconfig の DYNE_RANGE_AUTO / DYNE_IMU_AUTO)。設定は ctl 命令で変える:
 *   {"cmd":"range", ...}  測距の設定 (dyne_range.c の冒頭)
 *   {"cmd":"pan",   ...}  サーボの首振りの設定
 *   {"cmd":"imu",   ...}  IMU の設定 (dyne_imu.c の冒頭)
 * 状態は dyne/node/<id>/sense に発行する。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"
#include "driver/i2c_master.h"
#include "dyne_sense_core.h"

/* sensors.c: 共有の I2C バス (最初の呼び出しで作る。失敗は NULL) */
i2c_master_bus_handle_t dyne_i2c_bus(void);

/* ==== 測距 + 首振り (dyne_range.c) ==== */

/* 起動時に1回 (MQTT 開始の後)。センサを探して見つかれば測距を始める */
void dyne_range_start(void);
/* ctl 命令 "range" / "pan" を処理したら true */
bool dyne_range_on_ctl(const char *cmd, const cJSON *root);

/* アクターへ渡す1フレーム (行の取り出し・左右の反転を済ませた区画) */
typedef struct {
    uint32_t seq;          /* フレームの通し番号 (1 から) */
    uint32_t sensor_id;    /* 首振りなら id_base + 位置 */
    int pos;               /* 首振りの位置 (固定なら -1) */
    int64_t t_local_us;    /* データが揃った時刻 (esp_timer) */
    int64_t t_sync_us;     /* 同じ時刻の同期した時刻 (未同期は 0) */
    uint32_t count, cols;
    int16_t d_mm[DYNE_RANGE_MAX_ZONES];
    uint8_t st[DYNE_RANGE_MAX_ZONES];
} dyne_range_frame_t;

/* 通し番号が *last_seq より新しいフレームがあれば最新の1つを out に写し、*last_seq を進めて true。
 * *skipped には飛ばしたフレームの数 */
bool dyne_range_take(dyne_range_frame_t *out, uint32_t *last_seq, uint32_t *skipped);

/* 形のアクターへ注入するセンサの一覧 (センサID・位置・向き [rad])。
 * 向き = 取り付けの向き (注入) + 首振りの位置の角度 (較正表、無ければ指令の角度から)。
 * 測距は位置の推定に使わない (CLAUDE.md)。戻り値: 個数 */
int dyne_range_sensors(uint32_t *id, double *x, double *y, double *yaw, int max);
/* アクターの CFG_ZONE_ROW に入れる値 (行を取り出して送るなら 0、全区画なら設定の行)。-1 = 注入しない */
int dyne_range_actor_zone_row(void);
/* センサが動いているか */
bool dyne_range_active(void);
/* アクターへ渡したフレームを dyne/node/<id>/range_fed に QoS 1 で発行する (PC で同じ入力を再生するため。
 * 2026-11-15: 実行の設定の replay.log のときだけ使う)。MQTT のアウトボックスが max_outbox バイトを超えていたら
 * 送らない。戻り値 0 = 積んだ、負 = 送らなかった (呼び手が数える) */
int dyne_range_publish_fed(const dyne_range_frame_t *f, uint64_t step, int max_outbox);

/* ==== IMU (dyne_imu.c) ==== */

void dyne_imu_start(void);
bool dyne_imu_on_ctl(const char *cmd, const cJSON *root);

/* アクターへ渡す1標本 (SI 単位: m/s²、rad/s) */
typedef struct {
    int64_t t_sync_us;     /* 同期した時刻 (未同期は esp_timer の時刻をそのまま。t_synced=false) */
    bool t_synced;
    double a[3], g[3];
} dyne_imu_sample_t;

/* 読み出し位置 *rd から最大 max 個を取り出す (古い順)。*lost にはため置きがあふれて失った数。
 * *rd が 0 なら今の位置から始める (それ以前の標本は渡さない) */
int dyne_imu_take(dyne_imu_sample_t *out, int max, uint64_t *rd, uint64_t *lost);
/* 最も新しい標本の同期した時刻 (無ければ 0) */
int64_t dyne_imu_newest_t(void);
bool dyne_imu_active(void);
