/*
 * 計算部品の保存区画の形式と選び方 (2026-11-11)。宣言と説明は dyne_actor_slot.h。
 * ESP-IDF に依存しない (host_test/actor_slot_test.c で PC 上でも検査する)。
 *
 * 見出し 128 バイトの並び (リトルエンディアン):
 *   0  4  識別子 "DYAC"        4  4  版 (=1)          8  4  seq
 *   12 4  wasm の長さ          16 32 wasm の SHA-256
 *   48 4  aot の長さ           52 32 aot の SHA-256
 *   84 24 名前                 108 16 予備 (0)        124 4 CRC32 (0..123)
 */
#include "dyne_actor_slot.h"

#include <string.h>

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t dyne_act_crc32(const uint8_t *d, size_t len)
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

static uint32_t align_up(uint32_t v)
{
    return (v + DYNE_ACT_SECTOR - 1) / DYNE_ACT_SECTOR * DYNE_ACT_SECTOR;
}

uint32_t dyne_act_wasm_off(const dyne_act_hdr_t *h)
{
    (void)h;
    return DYNE_ACT_SECTOR;
}

uint32_t dyne_act_aot_off(const dyne_act_hdr_t *h)
{
    return DYNE_ACT_SECTOR + align_up(h->wasm_size);
}

uint32_t dyne_act_span(uint32_t wasm_size, uint32_t aot_size)
{
    return DYNE_ACT_SECTOR + align_up(wasm_size) + align_up(aot_size);
}

void dyne_act_hdr_encode(uint8_t out[DYNE_ACT_HDR_LEN], const dyne_act_hdr_t *h)
{
    memset(out, 0, DYNE_ACT_HDR_LEN);
    put32(out + 0, DYNE_ACT_MAGIC);
    put32(out + 4, DYNE_ACT_VERSION);
    put32(out + 8, h->seq);
    put32(out + 12, h->wasm_size);
    memcpy(out + 16, h->wasm_sha, 32);
    put32(out + 48, h->aot_size);
    memcpy(out + 52, h->aot_sha, 32);
    memcpy(out + 84, h->name, DYNE_ACT_NAME_LEN);
    out[84 + DYNE_ACT_NAME_LEN - 1] = 0;
    put32(out + 124, dyne_act_crc32(out, 124));
}

int dyne_act_hdr_decode(const uint8_t in[DYNE_ACT_HDR_LEN], uint32_t slot_size, dyne_act_hdr_t *h)
{
    if (get32(in + 0) != DYNE_ACT_MAGIC) return -1;
    if (get32(in + 4) != DYNE_ACT_VERSION) return -2;
    if (get32(in + 124) != dyne_act_crc32(in, 124)) return -3;
    memset(h, 0, sizeof(*h));
    h->seq = get32(in + 8);
    h->wasm_size = get32(in + 12);
    memcpy(h->wasm_sha, in + 16, 32);
    h->aot_size = get32(in + 48);
    memcpy(h->aot_sha, in + 52, 32);
    memcpy(h->name, in + 84, DYNE_ACT_NAME_LEN);
    h->name[DYNE_ACT_NAME_LEN - 1] = 0;
    if (h->wasm_size == 0 && h->aot_size == 0) return -5;
    /* 桁あふれを避けて比べる (長さは高々区画の大きさ) */
    if (h->wasm_size > slot_size || h->aot_size > slot_size) return -4;
    if ((uint64_t)dyne_act_span(h->wasm_size, h->aot_size) > slot_size) return -4;
    return 0;
}

int dyne_act_order(const bool valid[2], const uint32_t seq[2], int order[2])
{
    int n = 0;
    if (valid[0] && valid[1]) {
        order[0] = seq[1] > seq[0] ? 1 : 0;
        order[1] = 1 - order[0];
        n = 2;
    } else if (valid[0]) {
        order[n++] = 0;
    } else if (valid[1]) {
        order[n++] = 1;
    }
    return n;
}

int dyne_act_write_slot(const bool valid[2], const uint32_t seq[2])
{
    int order[2];
    int n = dyne_act_order(valid, seq, order);
    if (n == 0) return 0;
    return 1 - order[0];
}

uint32_t dyne_act_next_seq(const bool valid[2], const uint32_t seq[2])
{
    uint32_t m = 0;
    if (valid[0] && seq[0] > m) m = seq[0];
    if (valid[1] && seq[1] > m) m = seq[1];
    return m + 1;
}

bool dyne_act_should_skip(uint32_t newest_seq, uint32_t ok_seq, uint32_t tries, uint32_t max_tries)
{
    if (newest_seq == ok_seq) return false; /* 一度合格したものは飛ばさない */
    return tries > max_tries;
}
