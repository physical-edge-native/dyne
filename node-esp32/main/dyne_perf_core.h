/*
 * 協調して動かしたときの記録 (送った・受けたメッセージとステップの時刻) を、実行中はメモリに貯め、実行後に
 * MQTT で送る形にする部品 (ESP-IDF に依存しない。PC の C コンパイラでも検査できる) (2026-11-26、GitHub issue #167)
 *
 * 記録の形式の正本: docs/design/2026-11-26-coop-perf-log.md。crates/dyne-sim2d の --perf-log と同じ JSON の
 * キー (k = tx / rx / step) を使うので、scripts/coop_perf.py がシミュレータの記録と同じように解析する。
 * 実機では、送るときにまとめた形 (下の dyne_perf_batch) で dyne/node/<idx>/perf へ出し、PC の
 * scripts/coop_perf.py --from-mqtt が1行1記録の形に戻す。
 *
 * 時刻はすべて同期した時刻 (1970-01-01 からの µs)。未同期なら 0。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

enum { DYNE_PERF_TX = 1, DYNE_PERF_RX = 2, DYNE_PERF_STEP = 3 };

/* 1件の記録 (40 バイト)。tx: a = 送った時刻。rx: a = 送り手の時刻 (ヘッダの時刻印)、b = 受けた時刻。
 * step: a = 開始、b = 全界面の値が揃った時刻、c = 計算の終わり */
typedef struct {
    uint8_t kind;
    uint8_t resend;  /* tx: 再送・重複への応答なら 1 */
    uint16_t bytes;  /* 配送単位の大きさ (外箱を除く) */
    uint32_t iface;
    uint32_t step;
    uint32_t pad;
    int64_t a, b, c;
} dyne_perf_rec_t;

typedef struct {
    dyne_perf_rec_t *rec; /* 呼び出し側が用意する領域 (NULL なら記録しない) */
    uint32_t cap;         /* 領域に入る記録の数 */
    uint32_t n;           /* 入れた数 */
    uint32_t dropped;     /* 領域があふれて捨てた数 */
} dyne_perf_t;

void dyne_perf_init(dyne_perf_t *p, dyne_perf_rec_t *mem, uint32_t cap);
void dyne_perf_tx(dyne_perf_t *p, uint32_t iface, uint32_t step, int64_t t, uint32_t bytes, bool resend);
void dyne_perf_rx(dyne_perf_t *p, uint32_t iface, uint32_t step, int64_t t_sent, int64_t t, uint32_t bytes);
void dyne_perf_step(dyne_perf_t *p, uint32_t step, int64_t t0, int64_t t1, int64_t t2);

/* 配送単位 (枠つきメッセージ列: [長さ 4 バイト][メッセージ]...) の最初のメッセージのヘッダの時刻印 (オフセット 16、
 * リトルエンディアン 8 バイト)。短すぎれば 0 */
int64_t dyne_perf_unit_t_us(const uint8_t *unit, size_t len);

/* 1件を JSON の1つのオブジェクトにする (dyne-sim2d の --perf-log と同じキー)。戻り値: 書いた長さ、入らなければ -1 */
int dyne_perf_format(const dyne_perf_rec_t *r, int node, char *out, size_t n);

/* i0 番目から最大 max_n 件を、1つのメッセージにまとめる (件数 n は書き終えてから分かるので最後に置く):
 *   {"node":N,"steps":S,"i0":i0,"total":T,"dropped":D,"recs":[{...},...],"n":k}
 * out に入るだけ入れ、入れた件数を *done に返す (0 件でも閉じた JSON を書く)。戻り値: 書いた長さ、out が小さすぎれば -1 */
int dyne_perf_batch(const dyne_perf_t *p, int node, uint32_t steps, uint32_t i0, uint32_t max_n,
                    char *out, size_t n, uint32_t *done);
