/*
 * センサまわりの計算部分 (ESP-IDF に依存しない。PC の C コンパイラでもビルドできる) (2026-11-13)
 *
 * 説明: docs/2026-11-13-firmware-nodes.md
 *
 *  1. 測距フレーム (VL53L5CX の 4×4 / 8×8 区画) から、ワイヤ形式v2 の RANGE_FRAME
 *     (crates/dyne-wire/src/lib.rs、種別4) の枠つきメッセージを作る。行の取り出しと左右の反転もここ。
 *  2. サーボで首を振る (PAN4H) の手順: 位置の順番、常に同じ向き (角度の小さい側) から近づける
 *     (歯車のガタ対策)、動いている間とその直後のフレームを捨てる、落ち着いたら PWM を止める。
 *  3. IMU (LSM6DSOX) の FIFO (ため置き) の1語の解釈と、センサの時刻印 (25µs 刻みの計数) を
 *     ESP32 の esp_timer (起動からの µs) へ対応づける直線の当てはめ。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ==================== 1. 測距フレーム ==================== */

#define DYNE_RANGE_MAX_ZONES 64
#define DYNE_RANGE_ZONE_LEN  16  /* dyne-wire の ZONE_LEN: 距離 f64 [m]、状態値 u32、予備 u32 */
#define DYNE_V2_KIND_RANGE   4   /* dyne-wire の KIND_RANGE_FRAME */
#define DYNE_V2_QTY_NONE     255

/*
 * センサの区画の並び (nz = 16 または 64、行優先) から、送る区画を選ぶ。
 *   row  = 0..辺-1 ならその1行だけ (count = cols = 辺)。-1 なら全区画 (count = nz, cols = 辺)
 *   flip = true なら各行の左右を反転する (アクターの区画0 = 左端 (正の角度) に合わせる)
 * 戻り値: 0 / -1 (nz・row が不正)
 */
int dyne_range_select(const int16_t *d_mm, const uint8_t *st, int nz, int row, bool flip,
                      int16_t *d_out, uint8_t *st_out, uint32_t *count, uint32_t *cols);

/* 1つの RANGE_FRAME を枠つき ([u32 長さ][メッセージ]) で out に書く。距離 [m] = mm / 1000.0。
 * 戻り値: 書いたバイト数 / -1 (容量不足・cols が不正) */
int dyne_range_frame_build(uint8_t *out, size_t cap, uint32_t sensor_id, uint64_t step, uint64_t t_us,
                           const int16_t *d_mm, const uint8_t *st, uint32_t count, uint32_t cols);

/* ==================== 2. サーボの首振り ==================== */

#define DYNE_PAN_MAX_POS 16

typedef struct {
    int n_pos;                       /* 位置の数 (1..16) */
    float cmd_deg[DYNE_PAN_MAX_POS]; /* 各位置のサーボへの指令の角度 [度] (0..deg_max) */
    uint32_t id_base;                /* 位置 k のセンサID = id_base + k */
    float backlash_deg;              /* 小さい側から近づけるための手前の角度 [度] (0 = しない) */
    float ms_per_deg;                /* 移動の時間の見込み [ms/度] */
    uint32_t settle_ms;              /* 移動の後の落ち着きの時間 [ms] */
    uint32_t frames;                 /* 1位置で採るフレームの数 */
    uint32_t discard;                /* 落ち着いた後、さらに捨てるフレームの数 */
    uint32_t frame_period_us;        /* 測距の周期 [µs] (積分が動きと重なったフレームを捨てる判定) */
    uint32_t dwell_max_ms;           /* 1位置の最長の滞在 (フレームが来なくても次へ。0 = 無制限) */
    bool hold;                       /* 落ち着いた後も PWM を出し続ける (既定 false = 止める) */
    float deg_max;                   /* 指令の角度の上限 [度] (SG90 は 180) */
    uint32_t us_min, us_max;         /* 0 度・deg_max 度のパルス幅 [µs] */
} dyne_pan_cfg_t;

typedef enum { DYNE_PAN_IDLE = 0, DYNE_PAN_PRE, DYNE_PAN_MOVE, DYNE_PAN_DWELL } dyne_pan_state_t;

typedef struct {
    dyne_pan_cfg_t c;
    dyne_pan_state_t state;
    int pos;            /* いまの (または向かっている) 位置 */
    float cur_deg;      /* 最後に指令した角度 (負 = 不明) */
    float target_deg;
    int64_t t_done;     /* 動きが終わる見込みの時刻 [µs] (esp_timer) */
    uint32_t taken, skipped;
    uint64_t cycles;    /* 一巡した回数 */
    uint64_t n_accept, n_discard, n_timeout;
} dyne_pan_t;

/* サーボへの指示 (呼び出し側が LEDC で実行する) */
typedef struct {
    bool set;      /* 角度 deg のパルスを出す */
    float deg;
    bool off;      /* パルスを止める */
} dyne_pan_act_t;

/* 既定値 (PAN4H: 4方向 × 半区画 2.8度ずらし = 8位置) を入れる */
void dyne_pan_default(dyne_pan_cfg_t *c);
/* 設定の検査。戻り値 NULL = 正しい / 誤りの説明 */
const char *dyne_pan_check(const dyne_pan_cfg_t *c);
/* 首振りを位置0から始める。cur_deg < 0 (角度が不明) なら必ず手前から近づける */
void dyne_pan_start(dyne_pan_t *p, const dyne_pan_cfg_t *c, float cur_deg, int64_t now, dyne_pan_act_t *a);
/* 時間の経過 (こまめに呼ぶ) */
void dyne_pan_tick(dyne_pan_t *p, int64_t now, dyne_pan_act_t *a);
/* 測距フレームが届いた (t_ready = データが揃った時刻)。採るなら位置の番号、捨てるなら -1。
 * 1位置のフレームを採り終えたら次の位置への移動を始める (a に指示) */
int dyne_pan_on_frame(dyne_pan_t *p, int64_t t_ready, dyne_pan_act_t *a);
/* 角度 → パルス幅 [µs] */
uint32_t dyne_pan_pulse_us(const dyne_pan_cfg_t *c, float deg);

/* ==================== 3. IMU (LSM6DSOX) ==================== */

/* FIFO の1語 (7バイト: 札1 + 値6) の札 (上位5ビット) */
#define DYNE_LSM_TAG_GYRO   0x01
#define DYNE_LSM_TAG_ACCEL  0x02
#define DYNE_LSM_TAG_TEMP   0x03
#define DYNE_LSM_TAG_TS     0x04
#define DYNE_LSM_TAG_CFG    0x05

typedef struct {
    uint64_t tick;      /* センサの時刻印 (25µs 刻みの計数、桁あふれを補って 64 ビット) */
    int16_t a[3], g[3]; /* 生の値 (加速度・角速度) */
} dyne_imu_raw_t;

typedef struct {
    uint64_t wrap;      /* 時刻印の桁あふれの補正の状態 */
    bool have_ts, have_a, have_g;
    uint64_t ts;
    int16_t a[3], g[3];
    uint64_t n_words, n_samples, n_orphan, n_cfg;  /* 計数 (orphan = 時刻印の無い値) */
} dyne_imu_parse_t;

void dyne_imu_parse_init(dyne_imu_parse_t *p);
/* 32 ビットの計数を 64 ビットへ (前回から 2^31 未満しか進まない前提) */
uint64_t dyne_imu_unwrap(uint64_t *state, uint32_t raw);
/* FIFO の1語を入れる。標本 (時刻印 + 加速度 + 角速度) が揃ったら *out に書いて 1 を返す。
 * 並びは「時刻印 → 値」を前提にする (時刻印の語の後に来た加速度と角速度をその時刻印の標本とする) */
int dyne_imu_fifo_word(dyne_imu_parse_t *p, const uint8_t w[7], dyne_imu_raw_t *out);

/* LSM6DSOX の設定値 */
int dyne_lsm_odr_code(double hz);                /* 12.5..833 Hz → 1..7、それ以外 -1 */
double dyne_lsm_odr_hz(int code);
int dyne_lsm_fs_xl_bits(int fs_g);               /* ±2/4/8/16 g → CTRL1_XL の FS 欄 (2ビット)、不正 -1 */
double dyne_lsm_acc_sens_g(int fs_g);            /* g/LSB */
int dyne_lsm_fs_g_bits(int fs_dps);              /* ±250/500/1000/2000 dps → CTRL2_G の FS 欄、不正 -1 */
double dyne_lsm_gyro_sens_dps(int fs_dps);       /* dps/LSB */
/* INTERNAL_FREQ_FINE (符号つき8ビット) から時刻印の1刻み [µs] (資料の式 1/(40000·(1+0.0015·ff))) */
double dyne_lsm_us_per_tick(int8_t freq_fine);

/* センサの時刻印 → esp_timer の対応 (直近の組に直線を当てはめる) */
#define DYNE_IMU_CLK_N 64
typedef struct {
    int n, head;
    uint64_t tick[DYNE_IMU_CLK_N];
    int64_t loc[DYNE_IMU_CLK_N];
    double nominal;         /* 1刻みの公称値 [µs] (組が足りないとき) */
    double rate;            /* 当てはめた1刻み [µs] */
    double t_mean, l_mean;  /* 当てはめの重心 (tick と loc) */
    double resid_max;       /* 当てはめの残差の最大 [µs] (質の目安) */
} dyne_imu_clock_t;

void dyne_imu_clock_init(dyne_imu_clock_t *c, double us_per_tick);
/* 時刻印の値 tick を、esp_timer の時刻 local_us に読んだという組を足す */
void dyne_imu_clock_add(dyne_imu_clock_t *c, uint64_t tick, int64_t local_us);
/* tick の esp_timer の時刻 [µs]。組が無ければ 0 */
int64_t dyne_imu_clock_local(const dyne_imu_clock_t *c, uint64_t tick);

/* ==================== 4. IMU の融合の計算部品の実行 (2026-11-13 追記) ==================== */

/* 開始の時刻 t0 [µs] を PC が指定しなかったときの決め方: (今 + margin) 以降で最初の grid の倍数。
 * 全ノードがほぼ同時に start を受ければ同じ値になる (受けた時刻の差が grid の境目をまたがない限り) */
int64_t dyne_imu_auto_t0(int64_t now_us, int64_t margin_us, int64_t grid_us);

/* dyne/node/<id>/imu_fused の JSON {"step":s,"v":[v0,...]} を書く (docs/2026-11-13-imu-fusion.md §3.5)。
 * 値は %.17g (f64 を正確に戻せる桁)。有限でない値は Python の json が読める NaN / Infinity / -Infinity。
 * 戻り値: 書いた長さ / -1 (容量不足) */
int dyne_imu_fused_json(char *out, size_t cap, uint64_t step, const double *v, int n);
