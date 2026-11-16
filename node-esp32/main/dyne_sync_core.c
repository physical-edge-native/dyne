/*
 * ワイヤ形式v2 の経路のステップ同期の、受信の扱いの判定 (2026-11-15、GitHub issue #29)。説明は dyne_sync_core.h
 */
#include "dyne_sync_core.h"

void dyne_sync_echo_reset(dyne_sync_echo_t *e)
{
    e->echoed = false;
    e->last_echo_us = 0;
}

dyne_sync_rx_t dyne_sync_classify(uint64_t step, uint64_t msg_step, bool got, const dyne_sync_echo_t *e,
                                  int64_t now_us, int64_t min_gap_us)
{
    if (msg_step > step) return DYNE_SYNC_HOLD;
    if (msg_step < step) return DYNE_SYNC_REPLY_PAST;
    if (!got) return DYNE_SYNC_ACCEPT;
    /* 同じステップの重複 (crates/dyne-sim2d の on_deliver と同じ: 応答したことが無いか、最小間隔以上たった) */
    if (!e->echoed || now_us - e->last_echo_us >= min_gap_us) return DYNE_SYNC_REPLY_SAME;
    return DYNE_SYNC_DROP;
}

void dyne_sync_echo_mark(dyne_sync_echo_t *e, int64_t now_us)
{
    e->echoed = true;
    e->last_echo_us = now_us;
}

bool dyne_sync_stall_due(int64_t wait_start_us, int64_t now_us, int64_t stall_us, uint32_t n_reported)
{
    if (stall_us <= 0) return false;
    return now_us - wait_start_us >= stall_us * ((int64_t)n_reported + 1);
}
