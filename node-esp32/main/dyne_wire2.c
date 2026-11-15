/*
 * ワイヤ形式v2 の外箱の C 実装 (2026-11-08)。仕様は dyne_wire2.h と
 * crates/dyne-wire/src/envelope.rs。バイト順はリトルエンディアンを明示的に組む
 * (CPU のバイト順や構造体の詰め方に依存しない)。
 */
#include <string.h>
#include "dyne_wire2.h"
#include "dyne_wire2_golden.h"

static const uint8_t ENV_MAGIC[4] = { 'D', 'Y', 'N', '2' };

static void put32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}
static void put64(uint8_t *b, uint64_t v)
{
    put32(b, (uint32_t)v);
    put32(b + 4, (uint32_t)(v >> 32));
}
static uint32_t get32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static uint64_t get64(const uint8_t *b)
{
    return (uint64_t)get32(b) | ((uint64_t)get32(b + 4) << 32);
}

uint32_t dyne_env_crc32(const uint8_t *d, size_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) {
            uint32_t m = -(c & 1u);
            c = (c >> 1) ^ (0xEDB88320u & m);
        }
    }
    return ~c;
}

bool dyne_v2_routed_by_iface(uint8_t kind)
{
    return kind == DYNE_V2_KIND_IFACE_SCALAR || kind == DYNE_V2_KIND_IFACE_BAND
        || kind == DYNE_V2_KIND_RAYS;
}

int dyne_v2_peek(const uint8_t *raw, size_t len, uint8_t *kind, uint32_t *src, uint64_t *step)
{
    if (len < DYNE_V2_HDR_LEN || raw[0] != DYNE_V2_VERSION) return DYNE_ENV_E_UNIT;
    const uint8_t *p = raw + DYNE_V2_HDR_LEN;
    size_t rest = len - DYNE_V2_HDR_LEN;
    uint64_t plen;
    switch (raw[1]) {
    case DYNE_V2_KIND_IFACE_SCALAR: plen = 16; break;
    case DYNE_V2_KIND_POSITION:     plen = 40; break;
    case DYNE_V2_KIND_HEARSAY:      plen = 32; break;
    case DYNE_V2_KIND_IFACE_BAND: {
        if (rest < 8) return DYNE_ENV_E_UNIT;
        uint32_t count = get32(p), per_point = get32(p + 4);
        if (per_point == 0) return DYNE_ENV_E_UNIT;
        plen = 8 + 8ull * count * per_point;
        break;
    }
    case DYNE_V2_KIND_RANGE_FRAME: {
        if (rest < 8) return DYNE_ENV_E_UNIT;
        uint32_t count = get32(p), cols = get32(p + 4);
        if (cols == 0 || count % cols != 0) return DYNE_ENV_E_UNIT;
        plen = 8 + 16ull * count;
        break;
    }
    case DYNE_V2_KIND_RAYS: {
        if (rest < 8) return DYNE_ENV_E_UNIT;
        plen = 8 + 40ull * get32(p);
        break;
    }
    default: return DYNE_ENV_E_UNIT;
    }
    if (plen != rest) return DYNE_ENV_E_UNIT;
    *kind = raw[1];
    *src = get32(raw + 4);
    *step = get64(raw + 8);
    return 0;
}

int dyne_env_check_unit(const uint8_t *payload, size_t len, uint32_t iface, uint64_t step)
{
    size_t pos = 0;
    int n = 0;
    while (pos < len) {
        if (pos + 4 > len) return DYNE_ENV_E_UNIT;
        size_t mlen = get32(payload + pos);
        size_t start = pos + 4;
        if (mlen == 0 || mlen > len - start) return DYNE_ENV_E_UNIT;
        uint8_t kind; uint32_t src; uint64_t mstep;
        if (dyne_v2_peek(payload + start, mlen, &kind, &src, &mstep) != 0) return DYNE_ENV_E_UNIT;
        if (!dyne_v2_routed_by_iface(kind) || src != iface || mstep != step) return DYNE_ENV_E_UNIT;
        pos = start + mlen;
        n++;
    }
    return n > 0 ? n : DYNE_ENV_E_UNIT;
}

int dyne_env_encode(uint8_t *out, size_t cap, const dyne_env_hdr_t *h,
                    const uint8_t *payload, size_t plen)
{
    if (plen > DYNE_ENV_MAX_PAYLOAD) return DYNE_ENV_E_OVERSIZE;
    if (dyne_env_check_unit(payload, plen, h->iface, h->step) < 0) return DYNE_ENV_E_UNIT;
    size_t total = DYNE_ENV_HDR_LEN + plen + DYNE_ENV_CRC_LEN;
    if (cap < total) return DYNE_ENV_E_FULL;
    memcpy(out, ENV_MAGIC, 4);
    out[4] = DYNE_ENV_VERSION;
    out[5] = 0; out[6] = 0; out[7] = 0;
    put32(out + 8, h->sender);
    put32(out + 12, h->seq);
    put32(out + 16, h->iface);
    put32(out + 20, (uint32_t)plen);
    put64(out + 24, h->step);
    memmove(out + DYNE_ENV_HDR_LEN, payload, plen);
    size_t body = DYNE_ENV_HDR_LEN + plen;
    put32(out + body, dyne_env_crc32(out, body));
    return (int)total;
}

int dyne_env_decode(const uint8_t *dgram, size_t len, dyne_env_hdr_t *h,
                    const uint8_t **payload, size_t *plen)
{
    if (len > DYNE_ENV_MAX_DATAGRAM) return DYNE_ENV_E_OVERSIZE;
    if (len < DYNE_ENV_HDR_LEN + DYNE_ENV_CRC_LEN) return DYNE_ENV_E_TRUNCATED;
    if (memcmp(dgram, ENV_MAGIC, 4) != 0) return DYNE_ENV_E_MAGIC;
    size_t pl = get32(dgram + 20);
    if (pl > DYNE_ENV_MAX_PAYLOAD) return DYNE_ENV_E_OVERSIZE;
    if (DYNE_ENV_HDR_LEN + pl + DYNE_ENV_CRC_LEN != len) return DYNE_ENV_E_TRUNCATED;
    size_t body = DYNE_ENV_HDR_LEN + pl;
    if (dyne_env_crc32(dgram, body) != get32(dgram + body)) return DYNE_ENV_E_CRC;
    if (dgram[4] != DYNE_ENV_VERSION || dgram[5] || dgram[6] || dgram[7]) return DYNE_ENV_E_VERSION;
    h->sender = get32(dgram + 8);
    h->seq = get32(dgram + 12);
    h->iface = get32(dgram + 16);
    h->step = get64(dgram + 24);
    if (dyne_env_check_unit(dgram + DYNE_ENV_HDR_LEN, pl, h->iface, h->step) < 0) return DYNE_ENV_E_UNIT;
    *payload = dgram + DYNE_ENV_HDR_LEN;
    *plen = pl;
    return 0;
}

bool dyne_env_selftest(void)
{
    /* 照合用の入力を C で組み立てる (dyne_wire2_golden.h の説明と同じ) */
    uint8_t unit[52];
    put32(unit, 48);                    /* 枠: メッセージ長 */
    uint8_t *m = unit + 4;
    m[0] = DYNE_V2_VERSION; m[1] = DYNE_V2_KIND_IFACE_BAND; m[2] = 0; m[3] = 0;
    put32(m + 4, 5);                    /* src = 界面5 */
    put64(m + 8, 42);                   /* step */
    put64(m + 16, 0);                   /* 時刻印 */
    put32(m + 24, 2);                   /* 点の数 */
    put32(m + 28, 1);                   /* 1点あたりの量の数 */
    double v0 = 1.0, v1 = -2.5;
    uint64_t b0, b1;
    memcpy(&b0, &v0, 8);
    memcpy(&b1, &v1, 8);
    put64(m + 32, b0);
    put64(m + 40, b1);

    dyne_env_hdr_t h = { .sender = 3, .seq = 7, .iface = 5, .step = 42 };
    uint8_t out[128];
    int n = dyne_env_encode(out, sizeof(out), &h, unit, sizeof(unit));
    if (n != (int)sizeof(DYNE_ENV_GOLDEN_BYTES)) return false;
    if (memcmp(out, DYNE_ENV_GOLDEN_BYTES, (size_t)n) != 0) return false;

    dyne_env_hdr_t h2;
    const uint8_t *pl;
    size_t pn;
    if (dyne_env_decode(DYNE_ENV_GOLDEN_BYTES, sizeof(DYNE_ENV_GOLDEN_BYTES), &h2, &pl, &pn) != 0)
        return false;
    if (h2.sender != 3 || h2.seq != 7 || h2.iface != 5 || h2.step != 42 || pn != sizeof(unit)
        || memcmp(pl, unit, pn) != 0)
        return false;
    /* どの1ビットの破損も受理しない */
    for (int i = 0; i < n; i++) {
        for (int bit = 0; bit < 8; bit++) {
            out[i] ^= (uint8_t)(1u << bit);
            int rc = dyne_env_decode(out, (size_t)n, &h2, &pl, &pn);
            out[i] ^= (uint8_t)(1u << bit);
            if (rc == 0) return false;
        }
    }
    /* 上限を超える中身は送らない (長さの検査が先なので中身は読まれない。
     * 大きな検査用バッファを静的に取ると既存の1次元経路の空きメモリを減らすので取らない) */
    if (dyne_env_encode(out, sizeof(out), &h, unit, DYNE_ENV_MAX_PAYLOAD + 1) != DYNE_ENV_E_OVERSIZE)
        return false;
    return true;
}
