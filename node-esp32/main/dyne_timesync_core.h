/*
 * 時刻同期の計算部分 (ESP-IDF に依存しない。PC の C コンパイラでも検査できる) (2026-11-11)
 *
 * 説明: docs/2026-11-11-timesync-ota.md
 *
 * やっていること:
 *  1. NTP (ネットワーク時刻プロトコル) の要求・応答の48バイトのパケットの組み立てと解釈。
 *     PC の時刻サーバ (scripts/dyne_ntp_server.py) と、往復の時刻印を交換する。
 *  2. 往復1回の4つの時刻印 (t1=ノード送信, t2=PC受信, t3=PC送信, t4=ノード受信) から
 *     ずれ (PC の時計 - ノードの時計) と往復時間を求める (NTP と同じ式)。
 *  3. 直近の標本 (最大 DYNE_TS_WINDOW 個) に直線を当てはめ、ずれとその変化率
 *     (水晶の周波数の誤差 = ドリフト、ppm) を推定する。往復時間が大きい標本は捨てる。
 *  4. 推定から「同期した時刻の次の整数秒」がノードの時計で何時かを求める
 *     (試験モードのピン切り替えに使う)。
 *
 * ノードの時計 = esp_timer_get_time() (起動からの µs、単調増加)。
 * 同期した時刻 = PC の時計 (1970-01-01 からの µs。PC 側スクリプトが基準)。
 * 同期した時刻 = ノードの時計 + ずれ(ノードの時計)。
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define DYNE_NTP_PKT_LEN 48
/* 1900-01-01 (NTP の起点) から 1970-01-01 (Unix の起点) までの秒数 */
#define DYNE_NTP_UNIX_DELTA_S 2208988800ULL

/* NTP の64ビット時刻印 (上位32ビット=秒, 下位32ビット=秒の端数) と Unix µs の変換 */
uint64_t dyne_ntp_from_unix_us(int64_t unix_us);
int64_t dyne_ntp_to_unix_us(uint64_t ntp);

/* 要求パケット (版4, 依頼者モード)。送信時刻の欄に cookie (ノードの時計 µs) をそのまま入れる。
 * サーバはこれを応答の「依頼側の送信時刻」の欄へそのまま写す (NTP の規約) ので、
 * 応答と要求の対応づけに使う */
void dyne_ntp_make_request(uint8_t pkt[DYNE_NTP_PKT_LEN], uint64_t cookie);

/* 応答の解釈。戻り値 0 = 成功 / 負 = 不正 (-1 長さ, -2 モード, -3 階層0=拒否, -4 時刻印0)
 * cookie = 依頼側の送信時刻の欄、t2 = サーバ受信、t3 = サーバ送信 (Unix µs) */
int dyne_ntp_parse_reply(const uint8_t *pkt, size_t len, uint64_t *cookie,
                         int64_t *t2_us, int64_t *t3_us);

/* 往復1回の標本 */
typedef struct {
    int64_t offset_us; /* ずれ = ((t2 - t1) + (t3 - t4)) / 2  (サーバの時計 - ノードの時計) */
    int64_t rtt_us;    /* 往復時間 = (t4 - t1) - (t3 - t2) */
} dyne_ts_sample_t;

void dyne_ts_sample(int64_t t1_local, int64_t t2_srv, int64_t t3_srv, int64_t t4_local,
                    dyne_ts_sample_t *s);

/* 推定結果 (読み手が排他区間で写して使う小さな構造体) */
typedef struct {
    bool ready;
    int64_t base_us;  /* ずれの基準値 (最初の標本)。大きな値を double に入れないため */
    int64_t t_ref;    /* 当てはめの基準時刻 (ノードの時計) */
    double a_us;      /* t_ref でのずれ - base_us */
    double b;         /* ずれの変化率 (無次元。1e-6 = 1 ppm) */
} dyne_ts_fit_t;

/* ノードの時計 t_local でのずれ */
int64_t dyne_ts_fit_offset(const dyne_ts_fit_t *f, int64_t t_local);
/* ノードの時計 → 同期した時刻 / その逆 */
int64_t dyne_ts_fit_to_sync(const dyne_ts_fit_t *f, int64_t t_local);
int64_t dyne_ts_fit_to_local(const dyne_ts_fit_t *f, int64_t t_sync);
/* now_local より margin_us 以上先の、同期した時刻で period_us の整数倍になる時点を
 * ノードの時計で返す (period_us = 1000000 で整数秒)。sync_out に同期した時刻 */
int64_t dyne_ts_fit_next_tick(const dyne_ts_fit_t *f, int64_t now_local, int64_t period_us,
                              int64_t margin_us, int64_t *sync_out);

#define DYNE_TS_WINDOW 32

typedef struct {
    int n, head;
    int64_t t[DYNE_TS_WINDOW];   /* 標本の時刻 (ノードの時計) */
    double o[DYNE_TS_WINDOW];    /* 標本のずれ - base_us */
    int64_t rtt[DYNE_TS_WINDOW];
    int reject_run;              /* 続けて捨てた数 (経路が恒常的に遅くなったら受け入れ直す) */
    uint32_t accepted, rejected;
    dyne_ts_fit_t fit;
} dyne_ts_model_t;

void dyne_ts_model_init(dyne_ts_model_t *m);
/* 標本を加える。t_local = 標本の時刻 (t1 と t4 の中点を渡す)。
 * 戻り値 1 = 採用 / 0 = 捨てた。resid_us に「加える前の推定」との差を返す
 * (推定が無いときは 0)。これが同期の質の指標 */
int dyne_ts_model_add(dyne_ts_model_t *m, int64_t t_local, const dyne_ts_sample_t *s,
                      int64_t *resid_us);
/* 窓の中の最小の往復時間 (標本が無ければ -1) */
int64_t dyne_ts_model_min_rtt(const dyne_ts_model_t *m);
