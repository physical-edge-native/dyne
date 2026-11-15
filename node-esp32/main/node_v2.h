/*
 * 場のアクター (メールボックス方式) をワイヤ形式v2で同期させる経路 (2026-11-08)。
 * 実装と設定注入の形は node_v2.c の冒頭。
 */
#pragma once

#include <stdbool.h>
#include "wasm_export.h"

/* 積み込んだ wasm がメールボックス方式の差込口一式を持てば本経路を有効にして true */
bool dyne_v2_bind(wasm_module_inst_t inst, wasm_exec_env_t env);
/* 本経路が有効か (false なら既存の1次元の経路) */
bool dyne_v2_active(void);
/* 本経路の設定注入 (dyne/cfg/<id>) */
void dyne_v2_on_cfg(const char *payload, int len);
/* 本経路の BSP 実行 (start 受信時) */
void dyne_v2_run_bsp(void);
