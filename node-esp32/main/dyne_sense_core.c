/*
 * センサまわりの計算部分 (2026-11-13)。仕様は dyne_sense_core.h。
 * バイト順はリトルエンディアンを明示的に組む (dyne_wire2.c と同じ書き方)。
 */
#include <string.h>
#include <math.h>
#include "dyne_sense_core.h"

/* ==================== 1. 測距フレーム ==================== */

static void put32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}
static void put64(uint8_t *b, uint64_t v)
{
    put32(b, (uint32_t)v);
    put32(b + 4, (uint32_t)(v >> 32));
}

int dyne_range_select(const int16_t *d_mm, const uint8_t *st, int nz, int row, bool flip,
                      int16_t *d_out, uint8_t *st_out, uint32_t *count, uint32_t *cols)
{
    int side = nz == 16 ? 4 : nz == 64 ? 8 : 0;
    if (side == 0 || row < -1 || row >= side) return -1;
    int r0 = row < 0 ? 0 : row, r1 = row < 0 ? side : row + 1;
    int k = 0;
    for (int r = r0; r < r1; r++) {
        for (int c = 0; c < side; c++) {
            int src = r * side + (flip ? side - 1 - c : c);
            d_out[k] = d_mm[src];
            st_out[k] = st[src];
            k++;
        }
    }
    *count = (uint32_t)k;
    *cols = (uint32_t)side;
    return 0;
}

int dyne_range_frame_build(uint8_t *out, size_t cap, uint32_t sensor_id, uint64_t step, uint64_t t_us,
                           const int16_t *d_mm, const uint8_t *st, uint32_t count, uint32_t cols)
{
    if (cols == 0 || count % cols != 0 || count > DYNE_RANGE_MAX_ZONES) return -1;
    size_t mlen = 24 + 8 + (size_t)DYNE_RANGE_ZONE_LEN * count;
    if (cap < 4 + mlen) return -1;
    put32(out, (uint32_t)mlen);
    uint8_t *m = out + 4;
    m[0] = 2;                    /* 版 */
    m[1] = DYNE_V2_KIND_RANGE;   /* 種別 */
    m[2] = DYNE_V2_QTY_NONE;     /* 物理量 (指定なし。シミュレータと同じ) */
    m[3] = 0;
    put32(m + 4, sensor_id);     /* src = センサID */
    put64(m + 8, step);
    put64(m + 16, t_us);
    put32(m + 24, count);
    put32(m + 28, cols);
    uint8_t *z = m + 32;
    for (uint32_t i = 0; i < count; i++) {
        double dm = (double)d_mm[i] / 1000.0;
        uint64_t bits;
        memcpy(&bits, &dm, 8);
        put64(z, bits);
        put32(z + 8, st[i]);
        put32(z + 12, 0);
        z += DYNE_RANGE_ZONE_LEN;
    }
    return (int)(4 + mlen);
}

/* ==================== 2. サーボの首振り ==================== */

void dyne_pan_default(dyne_pan_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    /* PAN4H: 中央 90 度の左右に 22.5・67.5 度の4方向 (視野45度で180度を覆う) と、
     * それぞれを半区画 (45/8/2 = 2.8125 度) ずらした向き。シミュレータの場面
     * crates/dyne-sim2d/scenes/room6_move_pan4h.txt と同じ順 (位置 = 方向×2 + ずらし) */
    static const float deg[8] = { 22.5f, 25.3125f, 67.5f, 70.3125f, 112.5f, 115.3125f, 157.5f, 160.3125f };
    c->n_pos = 8;
    for (int k = 0; k < 8; k++) c->cmd_deg[k] = deg[k];
    c->id_base = 0;
    c->backlash_deg = 4.0f;
    c->ms_per_deg = 2.5f;      /* SG90 の公称 0.1 s/60度 (4.8V) より遅めに見込む */
    c->settle_ms = 150;
    c->frames = 3;
    c->discard = 0;
    c->frame_period_us = 66667; /* 15 Hz */
    c->dwell_max_ms = 1500;
    c->hold = false;
    c->deg_max = 180.0f;
    c->us_min = 500;
    c->us_max = 2400;
}

const char *dyne_pan_check(const dyne_pan_cfg_t *c)
{
    if (c->n_pos < 1 || c->n_pos > DYNE_PAN_MAX_POS) return "n_pos";
    if (!(c->deg_max > 0.0f)) return "deg_max";
    for (int k = 0; k < c->n_pos; k++)
        if (!(c->cmd_deg[k] >= 0.0f && c->cmd_deg[k] <= c->deg_max)) return "cmd_deg range";
    if (c->us_min < 200 || c->us_max > 3000 || c->us_min >= c->us_max) return "us_min/us_max";
    if (c->frames < 1) return "frames";
    if (c->backlash_deg < 0.0f || c->ms_per_deg < 0.0f) return "backlash/ms_per_deg";
    return NULL;
}

uint32_t dyne_pan_pulse_us(const dyne_pan_cfg_t *c, float deg)
{
    if (deg < 0.0f) deg = 0.0f;
    if (deg > c->deg_max) deg = c->deg_max;
    return c->us_min + (uint32_t)lroundf((float)(c->us_max - c->us_min) * deg / c->deg_max);
}

static int64_t move_us(const dyne_pan_t *p, float from, float to)
{
    float d = from < 0.0f ? p->c.deg_max : fabsf(to - from); /* 不明なら全域ぶん待つ */
    return (int64_t)(d * p->c.ms_per_deg * 1000.0f);
}

static void command(dyne_pan_t *p, float deg, dyne_pan_act_t *a)
{
    a->set = true;
    a->deg = deg;
    a->off = false;
    p->cur_deg = deg;
}

/* 位置 pos へ向かう。小さい側から近づけるため、今より小さい角度 (または不明) なら手前へ寄せる */
static void go_to(dyne_pan_t *p, int pos, int64_t now, dyne_pan_act_t *a)
{
    p->pos = pos;
    p->target_deg = p->c.cmd_deg[pos];
    float from = p->cur_deg;
    if (p->c.backlash_deg > 0.0f && (from < 0.0f || p->target_deg < from)) {
        float pre = p->target_deg - p->c.backlash_deg;
        if (pre < 0.0f) pre = 0.0f;
        p->state = DYNE_PAN_PRE;
        p->t_done = now + move_us(p, from, pre);
        command(p, pre, a);
    } else {
        p->state = DYNE_PAN_MOVE;
        p->t_done = now + move_us(p, from, p->target_deg) + (int64_t)p->c.settle_ms * 1000;
        command(p, p->target_deg, a);
    }
}

void dyne_pan_start(dyne_pan_t *p, const dyne_pan_cfg_t *c, float cur_deg, int64_t now, dyne_pan_act_t *a)
{
    memset(p, 0, sizeof(*p));
    memset(a, 0, sizeof(*a));
    p->c = *c;
    p->cur_deg = cur_deg;
    go_to(p, 0, now, a);
}

static void next_pos(dyne_pan_t *p, int64_t now, dyne_pan_act_t *a)
{
    int n = p->pos + 1;
    if (n >= p->c.n_pos) { n = 0; p->cycles++; }
    go_to(p, n, now, a);
}

void dyne_pan_tick(dyne_pan_t *p, int64_t now, dyne_pan_act_t *a)
{
    memset(a, 0, sizeof(*a));
    switch (p->state) {
    case DYNE_PAN_PRE:
        if (now >= p->t_done) {
            float from = p->cur_deg;
            p->state = DYNE_PAN_MOVE;
            p->t_done = now + move_us(p, from, p->target_deg) + (int64_t)p->c.settle_ms * 1000;
            command(p, p->target_deg, a);
        }
        break;
    case DYNE_PAN_MOVE:
        if (now >= p->t_done) {
            p->state = DYNE_PAN_DWELL;
            p->taken = 0;
            p->skipped = 0;
            if (!p->c.hold) a->off = true;
        }
        break;
    case DYNE_PAN_DWELL:
        if (p->c.dwell_max_ms > 0 && now - p->t_done >= (int64_t)p->c.dwell_max_ms * 1000) {
            p->n_timeout++;
            next_pos(p, now, a);
        }
        break;
    default:
        break;
    }
}

int dyne_pan_on_frame(dyne_pan_t *p, int64_t t_ready, dyne_pan_act_t *a)
{
    memset(a, 0, sizeof(*a));
    /* 積分の区間 (t_ready − 周期, t_ready] が、動きが終わる前と重なるフレームは捨てる */
    if (p->state != DYNE_PAN_DWELL || t_ready - (int64_t)p->c.frame_period_us < p->t_done) {
        p->n_discard++;
        return -1;
    }
    if (p->skipped < p->c.discard) {
        p->skipped++;
        p->n_discard++;
        return -1;
    }
    int pos = p->pos;
    p->n_accept++;
    if (++p->taken >= p->c.frames) next_pos(p, t_ready, a);
    return pos;
}

/* ==================== 3. IMU (LSM6DSOX) ==================== */

void dyne_imu_parse_init(dyne_imu_parse_t *p)
{
    memset(p, 0, sizeof(*p));
}

uint64_t dyne_imu_unwrap(uint64_t *state, uint32_t raw)
{
    uint64_t prev = *state;
    uint64_t v = (prev & ~(uint64_t)0xFFFFFFFFu) | raw;
    if (v + 0x80000000ull < prev) v += 0x100000000ull;       /* 前へ桁あふれ */
    else if (v > prev + 0x80000000ull && v >= 0x100000000ull) v -= 0x100000000ull; /* 少し戻った */
    *state = v;
    return v;
}

static int16_t s16(const uint8_t *b)
{
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

int dyne_imu_fifo_word(dyne_imu_parse_t *p, const uint8_t w[7], dyne_imu_raw_t *out)
{
    p->n_words++;
    uint8_t tag = w[0] >> 3;
    const uint8_t *d = w + 1;
    switch (tag) {
    case DYNE_LSM_TAG_TS: {
        if (p->have_a || p->have_g) p->n_orphan++; /* 前の時刻印の標本が揃わなかった */
        uint32_t raw = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
        p->ts = dyne_imu_unwrap(&p->wrap, raw);
        p->have_ts = true;
        p->have_a = p->have_g = false;
        return 0;
    }
    case DYNE_LSM_TAG_ACCEL:
        for (int k = 0; k < 3; k++) p->a[k] = s16(d + 2 * k);
        p->have_a = true;
        break;
    case DYNE_LSM_TAG_GYRO:
        for (int k = 0; k < 3; k++) p->g[k] = s16(d + 2 * k);
        p->have_g = true;
        break;
    case DYNE_LSM_TAG_CFG:
        p->n_cfg++;
        return 0;
    default:
        return 0;
    }
    if (!p->have_ts) {
        if (p->have_a && p->have_g) { p->n_orphan++; p->have_a = p->have_g = false; }
        return 0;
    }
    if (p->have_a && p->have_g) {
        out->tick = p->ts;
        memcpy(out->a, p->a, sizeof(out->a));
        memcpy(out->g, p->g, sizeof(out->g));
        p->have_a = p->have_g = false;
        p->have_ts = false; /* 1つの時刻印に1標本 */
        p->n_samples++;
        return 1;
    }
    return 0;
}

int dyne_lsm_odr_code(double hz)
{
    static const double t[] = { 12.5, 26, 52, 104, 208, 416, 833 };
    for (int i = 0; i < 7; i++)
        if (fabs(hz - t[i]) < 0.5) return i + 1;
    return -1;
}

double dyne_lsm_odr_hz(int code)
{
    static const double t[] = { 0, 12.5, 26, 52, 104, 208, 416, 833 };
    return code >= 1 && code <= 7 ? t[code] : 0.0;
}

int dyne_lsm_fs_xl_bits(int fs_g)
{
    switch (fs_g) { case 2: return 0; case 16: return 1; case 4: return 2; case 8: return 3; default: return -1; }
}

double dyne_lsm_acc_sens_g(int fs_g)
{
    switch (fs_g) { case 2: return 0.061e-3; case 4: return 0.122e-3; case 8: return 0.244e-3;
                    case 16: return 0.488e-3; default: return 0.0; }
}

int dyne_lsm_fs_g_bits(int fs_dps)
{
    switch (fs_dps) { case 250: return 0; case 500: return 1; case 1000: return 2; case 2000: return 3;
                      default: return -1; }
}

double dyne_lsm_gyro_sens_dps(int fs_dps)
{
    switch (fs_dps) { case 250: return 8.75e-3; case 500: return 17.5e-3; case 1000: return 35e-3;
                      case 2000: return 70e-3; default: return 0.0; }
}

double dyne_lsm_us_per_tick(int8_t freq_fine)
{
    return 1e6 / (40000.0 * (1.0 + 0.0015 * (double)freq_fine));
}

void dyne_imu_clock_init(dyne_imu_clock_t *c, double us_per_tick)
{
    memset(c, 0, sizeof(*c));
    c->nominal = us_per_tick;
    c->rate = us_per_tick;
}

static void refit(dyne_imu_clock_t *c)
{
    /* 最小二乗の直線 loc = l_mean + rate·(tick − t_mean)。値は最初の組からの差で計算する
     * (64 ビットの µs をそのまま double にすると桁が落ちるため) */
    int n = c->n;
    int first = (c->head - n + DYNE_IMU_CLK_N) % DYNE_IMU_CLK_N;
    uint64_t t0 = c->tick[first];
    int64_t l0 = c->loc[first];
    double st = 0, sl = 0;
    for (int i = 0; i < n; i++) {
        int k = (first + i) % DYNE_IMU_CLK_N;
        st += (double)(c->tick[k] - t0);
        sl += (double)(c->loc[k] - l0);
    }
    double tm = st / n, lm = sl / n, stt = 0, stl = 0;
    for (int i = 0; i < n; i++) {
        int k = (first + i) % DYNE_IMU_CLK_N;
        double dt = (double)(c->tick[k] - t0) - tm, dl = (double)(c->loc[k] - l0) - lm;
        stt += dt * dt;
        stl += dt * dl;
    }
    /* 時刻印の幅が短いうち (公称で 1 秒未満) は公称の刻みを使う (傾きが雑音で暴れるため) */
    double rate = (stt > 0 && (double)(c->tick[(c->head - 1 + DYNE_IMU_CLK_N) % DYNE_IMU_CLK_N] - t0) * c->nominal > 1e6)
                      ? stl / stt : c->nominal;
    /* 公称から ±2% を超える傾きは誤りとして公称を使う (資料の内部発振の精度は ±1.5% 程度) */
    if (!(fabs(rate / c->nominal - 1.0) < 0.02)) rate = c->nominal;
    c->rate = rate;
    c->t_mean = (double)t0 + tm;
    c->l_mean = (double)l0 + lm;
    double rmax = 0;
    for (int i = 0; i < n; i++) {
        int k = (first + i) % DYNE_IMU_CLK_N;
        double pred = lm + rate * ((double)(c->tick[k] - t0) - tm);
        double r = fabs((double)(c->loc[k] - l0) - pred);
        if (r > rmax) rmax = r;
    }
    c->resid_max = rmax;
}

void dyne_imu_clock_add(dyne_imu_clock_t *c, uint64_t tick, int64_t local_us)
{
    c->tick[c->head] = tick;
    c->loc[c->head] = local_us;
    c->head = (c->head + 1) % DYNE_IMU_CLK_N;
    if (c->n < DYNE_IMU_CLK_N) c->n++;
    refit(c);
}

int64_t dyne_imu_clock_local(const dyne_imu_clock_t *c, uint64_t tick)
{
    if (c->n == 0) return 0;
    return (int64_t)llround(c->l_mean + c->rate * ((double)tick - c->t_mean));
}

/* ==================== 4. IMU の融合の計算部品の実行 ==================== */

#include <stdio.h>

int64_t dyne_imu_auto_t0(int64_t now_us, int64_t margin_us, int64_t grid_us)
{
    int64_t t = now_us + margin_us;
    if (grid_us <= 0) return t;
    int64_t q = t / grid_us;
    if (q * grid_us < t) q++;
    return q * grid_us;
}

int dyne_imu_fused_json(char *out, size_t cap, uint64_t step, const double *v, int n)
{
    int k = snprintf(out, cap, "{\"step\":%llu,\"v\":[", (unsigned long long)step);
    if (k < 0 || (size_t)k >= cap) return -1;
    for (int i = 0; i < n; i++) {
        const char *sep = i ? "," : "";
        int m;
        if (isnan(v[i])) m = snprintf(out + k, cap - k, "%sNaN", sep);
        else if (isinf(v[i])) m = snprintf(out + k, cap - k, "%s%sInfinity", sep, v[i] < 0 ? "-" : "");
        else m = snprintf(out + k, cap - k, "%s%.17g", sep, v[i]);
        if (m < 0 || (size_t)(k + m) >= cap) return -1;
        k += m;
    }
    int m = snprintf(out + k, cap - k, "]}");
    if (m < 0 || (size_t)(k + m) >= cap) return -1;
    return k + m;
}
