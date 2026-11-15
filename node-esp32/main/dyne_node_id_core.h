/*
 * ノード番号の決め方の計算部分 (ESP-IDF に依存しない。PC の C コンパイラでも検査できる) (2026-11-15、GitHub issue #19)
 *
 * 説明: docs/2026-11-13-firmware-nodes.md §9.0.2
 *
 * ノード番号 (MQTT の dyne/cfg/<id>・dyne/ctl/<id>・dyne/node/<id>/... の <id>) は、起動時に次の順で決める:
 *   1. menuconfig の CONFIG_DYNE_NODE_ID が 0 以上なら、その値 (ビルドで固定。従来どおり)
 *   2. NVS (フラッシュの設定の保存領域) に番号が保存されていて、範囲 (0〜DYNE_NODE_ID_MAX) 内なら、その値
 *   3. どちらも無ければ Wi-Fi の MAC の末尾1バイト (0〜255。従来の CONFIG_DYNE_NODE_ID = -1 の動き)
 * 1つの像を全台に書き、番号の重なった基板だけ ctl 命令 node_id で NVS に番号を書けるようにするため。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 番号の範囲 (MAC の末尾1バイトと同じ 0〜255。PC 側の道具の前提を変えないため) */
#define DYNE_NODE_ID_MAX 255

typedef enum {
    DYNE_ID_SRC_CONFIG = 0, /* menuconfig (CONFIG_DYNE_NODE_ID >= 0) */
    DYNE_ID_SRC_NVS = 1,    /* NVS に保存した番号 */
    DYNE_ID_SRC_MAC = 2,    /* Wi-Fi の MAC の末尾1バイト */
} dyne_id_src_t;

/* "config" / "nvs" / "mac" (status の id_src の値) */
const char *dyne_id_src_name(dyne_id_src_t s);

/* 0〜DYNE_NODE_ID_MAX の整数か */
bool dyne_node_id_valid(double v);

/* 起動時の番号を決める。
 *   config_id: CONFIG_DYNE_NODE_ID (負 = 指定なし)
 *   nvs_has:   NVS に番号があった、nvs_id: その値 (範囲外なら無いものとして扱う)
 *   mac:       Wi-Fi の MAC (6バイト)
 *   src:       決め方を返す (NULL 可) */
int dyne_node_id_select(int config_id, bool nvs_has, int32_t nvs_id, const uint8_t mac[6], dyne_id_src_t *src);

/* MAC の文字列 ("24:0A:C4:12:34:56"、"24-0a-c4-12-34-56"、"240ac4123456"。大文字小文字は問わない) を読む */
bool dyne_mac_parse(const char *s, uint8_t out[6]);

/* ctl 命令 node_id の解釈の結果 */
typedef enum {
    DYNE_IDCMD_QUERY = 0,   /* id が無い: 今の番号を報告するだけ (mac があればその基板だけ) */
    DYNE_IDCMD_SET,         /* NVS に番号を書く (*out_id) */
    DYNE_IDCMD_CLEAR,       /* NVS の番号を消す (id = -1。次の起動から MAC の番号に戻る) */
    DYNE_IDCMD_NOT_MINE,    /* mac が自分のものではない: 何もせず、返事もしない */
    DYNE_IDCMD_ERR_NO_MAC,  /* id があるのに mac が無い (番号の重なった2台が両方書き換わるのを防ぐ) */
    DYNE_IDCMD_ERR_BAD_MAC, /* mac の書き方が違う */
    DYNE_IDCMD_ERR_BAD_ID,  /* id が -1 でも 0〜DYNE_NODE_ID_MAX の整数でもない */
} dyne_idcmd_t;

/* ctl 命令 node_id を解釈する。
 *   mac_str: 命令の "mac" (NULL = 無し)
 *   has_id:  命令に数値の "id" があった、id: その値
 *   my_mac:  この基板の Wi-Fi の MAC
 *   out_id:  SET のとき書く番号 (NULL 可) */
dyne_idcmd_t dyne_node_id_cmd(const char *mac_str, bool has_id, double id, const uint8_t my_mac[6], int32_t *out_id);

/* 失敗の符号 ("no mac" など。失敗でなければ NULL) */
const char *dyne_idcmd_err(dyne_idcmd_t c);
