/*
 * ノード番号の決め方の計算部分 (2026-11-15、GitHub issue #19)。宣言と説明は dyne_node_id_core.h。
 * ESP-IDF に依存しない。検査: node-esp32/host_test/node_id_test.c
 */
#include "dyne_node_id_core.h"

#include <string.h>

const char *dyne_id_src_name(dyne_id_src_t s)
{
    switch (s) {
    case DYNE_ID_SRC_CONFIG: return "config";
    case DYNE_ID_SRC_NVS: return "nvs";
    case DYNE_ID_SRC_MAC: return "mac";
    default: return "?";
    }
}

bool dyne_node_id_valid(double v)
{
    /* NaN は比較がすべて偽なので弾かれる */
    if (!(v >= 0.0 && v <= (double)DYNE_NODE_ID_MAX)) return false;
    return v == (double)(int32_t)v;
}

int dyne_node_id_select(int config_id, bool nvs_has, int32_t nvs_id, const uint8_t mac[6], dyne_id_src_t *src)
{
    dyne_id_src_t s;
    int id;
    if (config_id >= 0) {
        s = DYNE_ID_SRC_CONFIG;
        id = config_id;
    } else if (nvs_has && dyne_node_id_valid((double)nvs_id)) {
        s = DYNE_ID_SRC_NVS;
        id = (int)nvs_id;
    } else {
        s = DYNE_ID_SRC_MAC;
        id = mac[5];
    }
    if (src) *src = s;
    return id;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool dyne_mac_parse(const char *s, uint8_t out[6])
{
    if (!s) return false;
    size_t n = strlen(s);
    char sep = 0;
    if (n == 17) {
        sep = s[2];
        if (sep != ':' && sep != '-') return false;
    } else if (n != 12) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        const char *p = sep ? s + 3 * i : s + 2 * i;
        int hi = hexval(p[0]), lo = hexval(p[1]);
        if (hi < 0 || lo < 0) return false;
        if (sep && i < 5 && p[2] != sep) return false;
        out[i] = (uint8_t)(hi * 16 + lo);
    }
    return true;
}

dyne_idcmd_t dyne_node_id_cmd(const char *mac_str, bool has_id, double id, const uint8_t my_mac[6], int32_t *out_id)
{
    if (mac_str) {
        uint8_t m[6];
        if (!dyne_mac_parse(mac_str, m)) return DYNE_IDCMD_ERR_BAD_MAC;
        if (memcmp(m, my_mac, 6) != 0) return DYNE_IDCMD_NOT_MINE;
    }
    if (!has_id) return DYNE_IDCMD_QUERY;
    if (!mac_str) return DYNE_IDCMD_ERR_NO_MAC;
    if (id == -1.0) return DYNE_IDCMD_CLEAR;
    if (!dyne_node_id_valid(id)) return DYNE_IDCMD_ERR_BAD_ID;
    if (out_id) *out_id = (int32_t)id;
    return DYNE_IDCMD_SET;
}

const char *dyne_idcmd_err(dyne_idcmd_t c)
{
    switch (c) {
    case DYNE_IDCMD_ERR_NO_MAC: return "mac required";
    case DYNE_IDCMD_ERR_BAD_MAC: return "bad mac";
    case DYNE_IDCMD_ERR_BAD_ID: return "bad id (0-255, or -1 to clear)";
    default: return NULL;
    }
}
