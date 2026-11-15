/*
 * 形の区画のメッセージ (版1) の C 実装 (2026-11-13)。仕様は dyne_shape_msg.h。
 */
#include <string.h>
#include "dyne_shape_msg.h"

static void put16(uint8_t *b, uint16_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}
static void putf(uint8_t *b, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    put32(b, u);
}

size_t dyne_shape_msg_len(uint16_t nx, uint16_t ny)
{
    return DYNE_SHAPE_HDR_LEN + 12u * (size_t)nx * ny;
}

void dyne_shape_put(uint8_t *buf, size_t i, float v)
{
    putf(buf + DYNE_SHAPE_HDR_LEN + 4 * i, v);
}

size_t dyne_shape_finish(uint8_t *buf, const dyne_shape_hdr_t *h,
                         uint32_t (*crc32)(const uint8_t *, size_t))
{
    size_t body = 12u * (size_t)h->nx * h->ny;
    memcpy(buf, "DSF1", 4);
    buf[4] = DYNE_SHAPE_VERSION;
    buf[5] = DYNE_SHAPE_ENC_F32;
    put16(buf + 6, DYNE_SHAPE_HDR_LEN);
    put16(buf + 8, h->node);
    put16(buf + 10, 0);
    put32(buf + 12, h->step);
    put32(buf + 16, (uint32_t)h->t_us);
    put32(buf + 20, (uint32_t)(h->t_us >> 32));
    put16(buf + 24, h->ix0);
    put16(buf + 26, h->iy0);
    put16(buf + 28, h->nx);
    put16(buf + 30, h->ny);
    put16(buf + 32, h->gnx);
    put16(buf + 34, h->gny);
    putf(buf + 36, h->h);
    putf(buf + 40, h->x0);
    putf(buf + 44, h->y0);
    put32(buf + 48, crc32(buf + DYNE_SHAPE_HDR_LEN, body));
    return DYNE_SHAPE_HDR_LEN + body;
}
