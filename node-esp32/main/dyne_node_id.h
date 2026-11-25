/*
 * ノード番号を NVS (フラッシュの設定の保存領域) に置く (2026-11-15、GitHub issue #19)。
 * 決め方 (menuconfig → NVS → MAC の末尾1バイト) は dyne_node_id_core.h、説明は docs/2026-11-13-firmware-nodes.md §9.0.2。
 * dyne_node_id() 本体 (宣言は dyne_node.h) もここにある。
 */
#pragma once

#include <stdbool.h>
#include "cJSON.h"
#include "dyne_node_id_core.h"

/* NVS の初期化 (dyne_ops_boot_early) の後に1回: 番号を決めて覚える。以後、起動中は変わらない
 * (先に dyne_node_id() が呼ばれた場合もここを通る) */
void dyne_node_id_init(void);

/* 今の番号の決め方 */
dyne_id_src_t dyne_node_id_src(void);

/* online の status に足す項目 (JSON の断片、先頭カンマなし):
 * "id_src":"nvs"。NVS を書き換えて次の起動で番号が変わる場合は ,"id_next":N も */
int dyne_node_id_fields(char *buf, int len);

/* ctl 命令 node_id の処理。扱った命令なら true。
 *   {"cmd":"node_id"}                                     番号と決め方を報告 (dyne/ctl/all なら全台が返事)
 *   {"cmd":"node_id","mac":"02:00:00:12:34:56","id":5}   この MAC の基板の NVS に 5 を書く
 *   {"cmd":"node_id","mac":"...","id":-1}                 NVS の番号を消す (次の起動から MAC の番号)
 *   "reboot":true を足すと、番号が変わる場合はすぐ再起動する (既定は再起動しない。次の起動で効く)
 * 返事: dyne/node/<今の番号>/id/ack (QoS1、retain なし) */
bool dyne_node_id_on_ctl(const char *cmd, const cJSON *root);
