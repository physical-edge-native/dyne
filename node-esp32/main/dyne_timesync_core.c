/*
 * 時刻同期の計算部分 (2026-11-11)。宣言と説明は dyne_timesync_core.h。
 * ESP-IDF に依存しない (host_test/timesync_test.c で PC 上でも検査する)。
 */
#include "dyne_timesync_core.h"

#include <string.h>
#include <math.h>

/* 当てはめに直線 (ずれの変化率つき) を使い始める条件 */
#define FIT_MIN_N        3
#define FIT_MIN_SPAN_US  5000000LL   /* 5 秒 */
#define DRIFT_CLAMP      500e-6      /* 水晶の誤差は数十 ppm。これを超える推定は誤りとみなす */
/* 往復時間による選別: 窓の最小の 2 倍 + 2ms を超える標本は捨てる */
#define RTT_REJECT_MUL   2
#define RTT_REJECT_ADD   2000
#define REJECT_RUN_MAX   4           /* 4回続けて捨てたら受け入れる (経路の恒常的な変化への追従) */
#define STEP_RESET_US    50000       /* 推定との差が 50ms を超えたら時計の基準が変わったとみなし推定をやり直す */

static void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void put_be64(uint8_t *p, uint64_t v)
{
    put_be32(p, (uint32_t)(v >> 32));
    put_be32(p + 4, (uint32_t)v);
}

static uint64_t get_be64(const uint8_t *p)
{
    return ((uint64_t)get_be32(p) << 32) | get_be32(p + 4);
}

uint64_t dyne_ntp_from_unix_us(int64_t unix_us)
{
    uint64_t us = (uint64_t)unix_us;
    uint64_t sec = us / 1000000ULL + DYNE_NTP_UNIX_DELTA_S;
    uint64_t rem = us % 1000000ULL;
    uint64_t frac = (rem << 32) / 1000000ULL;
    return (sec << 32) | (frac & 0xFFFFFFFFULL);
}

int64_t dyne_ntp_to_unix_us(uint64_t ntp)
{
    uint64_t sec = ntp >> 32;
    uint64_t frac = ntp & 0xFFFFFFFFULL;
    uint64_t us = (frac * 1000000ULL + 0x80000000ULL) >> 32; /* 四捨五入 */
    return (int64_t)((sec - DYNE_NTP_UNIX_DELTA_S) * 1000000ULL + us);
}

void dyne_ntp_make_request(uint8_t pkt[DYNE_NTP_PKT_LEN], uint64_t cookie)
{
    memset(pkt, 0, DYNE_NTP_PKT_LEN);
    pkt[0] = (0 << 6) | (4 << 3) | 3; /* うるう秒の指示なし, 版4, 依頼者モード */
    put_be64(pkt + 40, cookie);        /* 送信時刻の欄 */
}

int dyne_ntp_parse_reply(const uint8_t *pkt, size_t len, uint64_t *cookie,
                         int64_t *t2_us, int64_t *t3_us)
{
    if (len < DYNE_NTP_PKT_LEN) return -1;
    if ((pkt[0] & 7) != 4) return -2;          /* サーバモードでない */
    if (pkt[1] == 0) return -3;                /* 階層0 = 拒否の通知 */
    uint64_t org = get_be64(pkt + 24), rx = get_be64(pkt + 32), tx = get_be64(pkt + 40);
    if (rx == 0 || tx == 0) return -4;
    *cookie = org;
    *t2_us = dyne_ntp_to_unix_us(rx);
    *t3_us = dyne_ntp_to_unix_us(tx);
    return 0;
}

void dyne_ts_sample(int64_t t1, int64_t t2, int64_t t3, int64_t t4, dyne_ts_sample_t *s)
{
    /* 和を先に取ると桁あふれしないよう、差どうしで計算する */
    int64_t d1 = t2 - t1, d2 = t3 - t4;
    s->offset_us = d1 / 2 + d2 / 2 + ((d1 % 2) + (d2 % 2)) / 2;
    s->rtt_us = (t4 - t1) - (t3 - t2);
}

int64_t dyne_ts_fit_offset(const dyne_ts_fit_t *f, int64_t t_local)
{
    double x = (double)(t_local - f->t_ref);
    return f->base_us + (int64_t)llround(f->a_us + f->b * x);
}

int64_t dyne_ts_fit_to_sync(const dyne_ts_fit_t *f, int64_t t_local)
{
    return t_local + dyne_ts_fit_offset(f, t_local);
}

int64_t dyne_ts_fit_to_local(const dyne_ts_fit_t *f, int64_t t_sync)
{
    /* t_local = t_sync - offset(t_local)。offset の変化は 1 秒で高々 500µs なので
     * 2 回の反復で µs 未満まで収まる */
    int64_t tl = t_sync - dyne_ts_fit_offset(f, f->t_ref);
    tl = t_sync - dyne_ts_fit_offset(f, tl);
    tl = t_sync - dyne_ts_fit_offset(f, tl);
    return tl;
}

int64_t dyne_ts_fit_next_tick(const dyne_ts_fit_t *f, int64_t now_local, int64_t period_us,
                              int64_t margin_us, int64_t *sync_out)
{
    int64_t s = dyne_ts_fit_to_sync(f, now_local + margin_us);
    int64_t next = (s / period_us + 1) * period_us;
    if (sync_out) *sync_out = next;
    return dyne_ts_fit_to_local(f, next);
}

void dyne_ts_model_init(dyne_ts_model_t *m)
{
    memset(m, 0, sizeof(*m));
}

int64_t dyne_ts_model_min_rtt(const dyne_ts_model_t *m)
{
    if (m->n == 0) return -1;
    int64_t r = m->rtt[0];
    for (int i = 1; i < m->n; i++) if (m->rtt[i] < r) r = m->rtt[i];
    return r;
}

static void refit(dyne_ts_model_t *m)
{
    int newest = (m->head + DYNE_TS_WINDOW - 1) % DYNE_TS_WINDOW;
    int64_t tref = m->t[newest];
    int64_t tmin = tref;
    for (int i = 0; i < m->n; i++) if (m->t[i] < tmin) tmin = m->t[i];
    dyne_ts_fit_t *f = &m->fit;
    f->t_ref = tref;
    if (m->n >= FIT_MIN_N && tref - tmin >= FIT_MIN_SPAN_US) {
        /* 最小二乗の直線 o = a + b x  (x = t - tref) */
        double sx = 0, so = 0;
        for (int i = 0; i < m->n; i++) { sx += (double)(m->t[i] - tref); so += m->o[i]; }
        double mx = sx / m->n, mo = so / m->n;
        double sxx = 0, sxo = 0;
        for (int i = 0; i < m->n; i++) {
            double dx = (double)(m->t[i] - tref) - mx;
            sxx += dx * dx;
            sxo += dx * (m->o[i] - mo);
        }
        double b = sxx > 0 ? sxo / sxx : 0.0;
        if (b > DRIFT_CLAMP) b = DRIFT_CLAMP;
        if (b < -DRIFT_CLAMP) b = -DRIFT_CLAMP;
        f->b = b;
        f->a_us = mo - b * mx;
    } else {
        f->b = 0.0;
        f->a_us = m->o[newest];
    }
    f->ready = true;
}

int dyne_ts_model_add(dyne_ts_model_t *m, int64_t t_local, const dyne_ts_sample_t *s,
                      int64_t *resid_us)
{
    int64_t resid = 0;
    if (m->fit.ready) resid = s->offset_us - dyne_ts_fit_offset(&m->fit, t_local);
    if (resid_us) *resid_us = resid;

    if (m->n >= 4) {
        int64_t minr = dyne_ts_model_min_rtt(m);
        if (s->rtt_us > RTT_REJECT_MUL * minr + RTT_REJECT_ADD && m->reject_run < REJECT_RUN_MAX) {
            m->reject_run++;
            m->rejected++;
            return 0;
        }
    }
    if (m->fit.ready && (resid > STEP_RESET_US || resid < -STEP_RESET_US)) {
        /* PC 側の時計の基準が変わった (サーバの再起動など)。推定をやり直す */
        uint32_t acc = m->accepted, rej = m->rejected;
        dyne_ts_model_init(m);
        m->accepted = acc;
        m->rejected = rej;
    }
    m->reject_run = 0;
    if (m->n == 0) m->fit.base_us = s->offset_us;
    m->t[m->head] = t_local;
    m->o[m->head] = (double)(s->offset_us - m->fit.base_us);
    m->rtt[m->head] = s->rtt_us;
    m->head = (m->head + 1) % DYNE_TS_WINDOW;
    if (m->n < DYNE_TS_WINDOW) m->n++;
    m->accepted++;
    refit(m);
    return 1;
}
