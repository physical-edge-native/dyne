/*
 * ワイヤ形式v2 の経路 (node_v2.c) のステップ同期の、受信の扱いの判定 (ESP-IDF に依存しない。PC の C コンパイラでも
 * 検査できる) (2026-11-15、GitHub issue #29)
 *
 * 規則の正本: docs/2026-11-15-sim2d-loss-jitter.md の「同期の受信の規則(シミュレータとファームで共通)」。
 * crates/dyne-sim2d の on_deliver と同じ規則にしてある (issue #26 の修正を移したもの)。
 *
 * 待ちの間 (ステップ step の値を全界面から集めている間) に、界面 k から ステップ msg_step の値が届いたとき:
 *   msg_step == step で、まだ受け取っていない      → 受け取る (受信メールボックスへ)
 *   msg_step == step で、すでに受け取っている       → 相手はまだこちらの値を受け取れていない (相手の再送)。
 *                                                     今のステップの控えを再送で応える。ただし界面ごとに、前の応答から
 *                                                     再送の最小間隔 (V2_RESEND_MIN_US = 20 ms) 以上たっていなければ捨てる
 *   msg_step >  step                               → 取り置く (将来分)
 *   msg_step <  step                               → そのステップの控えを再送で応える (過去分。従来どおり)
 * 応答の間隔の記録は、ステップの始め (送信) で界面ごとに消す。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 受信の扱い */
typedef enum {
    DYNE_SYNC_ACCEPT = 0,    /* 今のステップの初めての値: 受け取る */
    DYNE_SYNC_HOLD = 1,      /* 将来のステップ: 取り置く */
    DYNE_SYNC_REPLY_PAST = 2, /* 過去のステップ: そのステップの控えを再送で応える */
    DYNE_SYNC_REPLY_SAME = 3, /* 今のステップの重複: 今のステップの控えを再送で応える (issue #29) */
    DYNE_SYNC_DROP = 4,       /* 今のステップの重複だが、前の応答から最小間隔がたっていない: 捨てる */
} dyne_sync_rx_t;

/* 界面ごとの、今のステップの重複への応答の記録 */
typedef struct {
    bool echoed;          /* このステップで応答したか */
    int64_t last_echo_us; /* 最後の応答の時刻 (echoed のときだけ意味がある) */
} dyne_sync_echo_t;

/* ステップの始めに呼ぶ (応答の記録を消す) */
void dyne_sync_echo_reset(dyne_sync_echo_t *e);

/* 受信の扱いを決める (状態は変えない)。
 *   step:       待っているステップ
 *   msg_step:   届いた値のステップ
 *   got:        その界面からこのステップの値をすでに受け取ったか
 *   e:          その界面の応答の記録
 *   now_us:     今の時刻
 *   min_gap_us: 同じステップの重複への応答の最小間隔 (再送の最小間隔と同じ値を渡す) */
dyne_sync_rx_t dyne_sync_classify(uint64_t step, uint64_t msg_step, bool got, const dyne_sync_echo_t *e,
                                  int64_t now_us, int64_t min_gap_us);

/* DYNE_SYNC_REPLY_SAME で控えを送ったら呼ぶ */
void dyne_sync_echo_mark(dyne_sync_echo_t *e, int64_t now_us);

/* 止まっていることの報告の時期か。
 *   wait_start_us: このステップの待ちを始めた時刻
 *   stall_us:      報告の間隔 (0 以下 = 報告しない)
 *   n_reported:    このステップですでに報告した回数
 * 待ちが stall_us × (n_reported + 1) 以上続いたら true (報告は stall_us ごとに繰り返す) */
bool dyne_sync_stall_due(int64_t wait_start_us, int64_t now_us, int64_t stall_us, uint32_t n_reported);
