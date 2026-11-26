/* 協調して動かしたときの記録 (2026-11-26、GitHub issue #167)。説明は dyne_perf_core.h */
#include "dyne_perf_core.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

void dyne_perf_init(dyne_perf_t *p, dyne_perf_rec_t *mem, uint32_t cap)
{
    p->rec = mem;
    p->cap = mem ? cap : 0;
    p->n = 0;
    p->dropped = 0;
}

static dyne_perf_rec_t *slot(dyne_perf_t *p)
{
    if (!p->rec) return NULL;
    if (p->n >= p->cap) {
        p->dropped++;
        return NULL;
    }
    dyne_perf_rec_t *r = &p->rec[p->n++];
    memset(r, 0, sizeof(*r));
    return r;
}

void dyne_perf_tx(dyne_perf_t *p, uint32_t iface, uint32_t step, int64_t t, uint32_t bytes, bool resend)
{
    dyne_perf_rec_t *r = slot(p);
    if (!r) return;
    r->kind = DYNE_PERF_TX;
    r->resend = resend ? 1 : 0;
    r->bytes = (uint16_t)(bytes > 0xFFFF ? 0xFFFF : bytes);
    r->iface = iface;
    r->step = step;
    r->a = t;
}

void dyne_perf_rx(dyne_perf_t *p, uint32_t iface, uint32_t step, int64_t t_sent, int64_t t, uint32_t bytes)
{
    dyne_perf_rec_t *r = slot(p);
    if (!r) return;
    r->kind = DYNE_PERF_RX;
    r->bytes = (uint16_t)(bytes > 0xFFFF ? 0xFFFF : bytes);
    r->iface = iface;
    r->step = step;
    r->a = t_sent;
    r->b = t;
}

void dyne_perf_step(dyne_perf_t *p, uint32_t step, int64_t t0, int64_t t1, int64_t t2)
{
    dyne_perf_rec_t *r = slot(p);
    if (!r) return;
    r->kind = DYNE_PERF_STEP;
    r->step = step;
    r->a = t0;
    r->b = t1;
    r->c = t2;
}

int64_t dyne_perf_unit_t_us(const uint8_t *unit, size_t len)
{
    if (!unit || len < 4 + 24) return 0;
    uint64_t t = 0;
    for (int k = 7; k >= 0; k--) t = (t << 8) | unit[4 + 16 + k];
    return (int64_t)t;
}

int dyne_perf_format(const dyne_perf_rec_t *r, int node, char *out, size_t n)
{
    int w;
    switch (r->kind) {
    case DYNE_PERF_TX:
        w = snprintf(out, n, "{\"k\":\"tx\",\"node\":%d,\"iface\":%" PRIu32 ",\"step\":%" PRIu32 ",\"t\":%" PRId64
                     ",\"bytes\":%u,\"resend\":%s}",
                     node, r->iface, r->step, r->a, (unsigned)r->bytes, r->resend ? "true" : "false");
        break;
    case DYNE_PERF_RX:
        w = snprintf(out, n, "{\"k\":\"rx\",\"node\":%d,\"iface\":%" PRIu32 ",\"step\":%" PRIu32 ",\"t_sent\":%" PRId64
                     ",\"t\":%" PRId64 ",\"bytes\":%u}",
                     node, r->iface, r->step, r->a, r->b, (unsigned)r->bytes);
        break;
    case DYNE_PERF_STEP:
        w = snprintf(out, n, "{\"k\":\"step\",\"node\":%d,\"step\":%" PRIu32 ",\"t0\":%" PRId64 ",\"t1\":%" PRId64
                     ",\"t2\":%" PRId64 "}",
                     node, r->step, r->a, r->b, r->c);
        break;
    default:
        return -1;
    }
    return (w < 0 || (size_t)w >= n) ? -1 : w;
}

int dyne_perf_batch(const dyne_perf_t *p, int node, uint32_t steps, uint32_t i0, uint32_t max_n,
                    char *out, size_t n, uint32_t *done)
{
    *done = 0;
    char tail[64];
    int w = snprintf(out, n, "{\"node\":%d,\"steps\":%" PRIu32 ",\"i0\":%" PRIu32 ",\"total\":%" PRIu32
                     ",\"dropped\":%" PRIu32 ",\"recs\":[",
                     node, steps, i0, p->n, p->dropped);
    if (w < 0 || (size_t)w >= n) return -1;
    size_t pos = (size_t)w;
    uint32_t k = 0;
    char one[192];
    while (i0 + k < p->n && k < max_n) {
        int m = dyne_perf_format(&p->rec[i0 + k], node, one, sizeof(one));
        if (m < 0) return -1;
        /* 区切り + 1件 + 閉じ (最大 "],\"n\":4294967295}" の 20 バイト) が入るか */
        size_t need = (k ? 1 : 0) + (size_t)m + 24;
        if (pos + need >= n) break;
        if (k) out[pos++] = ',';
        memcpy(out + pos, one, (size_t)m);
        pos += (size_t)m;
        k++;
    }
    int t = snprintf(tail, sizeof(tail), "],\"n\":%" PRIu32 "}", k);
    if (t < 0 || pos + (size_t)t >= n) return -1;
    memcpy(out + pos, tail, (size_t)t + 1);
    *done = k;
    return (int)(pos + (size_t)t);
}
