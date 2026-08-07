/*
 * Dyne M1: ESP32 (ESPr Developer 32 Type-C) 上で heat1d アクターを WAMR で実行する。
 *
 * 目的:
 *  - M0 の WASM アクター (純計算体・変更なし) が実機の WAMR で動くことの確認
 *  - 限界レポート第2号の計測: RAM 使用量・フラッシュ使用量・1ステップ実行時間
 *
 * 問題設定は M0 の単一ノード参照解と同一:
 *  全域 [0,1] を 65 点 (h=1/64)、u(x,0)=sin(pi x)、両端 u=0、
 *  alpha=1, dt=1e-4, 2000 ステップ (t=0.2)。
 *  期待値 (PC の M0 実行と一致すべき値):
 *   L2=5.632e-5, Linf=8.027e-5
 *   u[16]=9.816824440839e-2, u[32]=1.388308626367e-1 (12桁表示)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pthread.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

#include "wasm_export.h"

/* main/CMakeLists.txt の EMBED_FILES で埋め込まれる WASM バイナリ */
extern const uint8_t wasm_start[] asm("_binary_heat1d_actor_wasm_start");
extern const uint8_t wasm_end[]   asm("_binary_heat1d_actor_wasm_end");

#define N_POINTS 65
#define STEPS    2000
#define ALPHA    1.0
#define DT       1e-4

static wasm_function_inst_t lookup(wasm_module_inst_t inst, const char *name)
{
    wasm_function_inst_t f = wasm_runtime_lookup_function(inst, name);
    if (!f) {
        printf("[m1] ERROR: export not found: %s\n", name);
    }
    return f;
}

/*
 * 注意: ESP-IDF の pthread_self() は「pthread として作られたタスク」以外から
 * 呼ばれると assert で停止する。WAMR は関数呼び出し時に内部で pthread_self() を
 * 使うため、WAMR の処理は必ず pthread 上で実行する (app_main の素の
 * main_task から直接呼ぶと落ちる。M1 実機で実際に踏んだ)。
 */
static void *iwasm_task(void *arg)
{
    printf("\n==== Dyne M1: WAMR on ESP32 ====\n");

    size_t heap_at_boot = esp_get_free_heap_size();
    printf("[m1] free heap at boot: %u bytes\n", (unsigned)heap_at_boot);

    /* ---- WAMR 初期化 (システムアロケータ使用) ---- */
    RuntimeInitArgs init_args;
    memset(&init_args, 0, sizeof(init_args));
    init_args.mem_alloc_type = Alloc_With_System_Allocator;
    if (!wasm_runtime_full_init(&init_args)) {
        printf("[m1] ERROR: wasm_runtime_full_init failed\n");
        return NULL;
    }

    /* 埋め込みバイナリはフラッシュ上 (書き換え不可) のため RAM へコピーする */
    uint32_t wasm_size = (uint32_t)(wasm_end - wasm_start);
    uint8_t *wasm_buf = malloc(wasm_size);
    if (!wasm_buf) {
        printf("[m1] ERROR: malloc for wasm buffer failed\n");
        return NULL;
    }
    memcpy(wasm_buf, wasm_start, wasm_size);
    printf("[m1] wasm module size: %u bytes\n", (unsigned)wasm_size);

    char error_buf[128];
    wasm_module_t module =
        wasm_runtime_load(wasm_buf, wasm_size, error_buf, sizeof(error_buf));
    if (!module) {
        printf("[m1] ERROR: load failed: %s\n", error_buf);
        return NULL;
    }

    /* stack 16KB (WASM側), ホスト管理ヒープ 0 (アクターは malloc しない) */
    wasm_module_inst_t inst =
        wasm_runtime_instantiate(module, 16 * 1024, 0, error_buf, sizeof(error_buf));
    if (!inst) {
        printf("[m1] ERROR: instantiate failed: %s\n", error_buf);
        printf("[m1] (線形メモリ128KBを確保できない場合はここで失敗する。"
               "限界レポートに記録すること)\n");
        return NULL;
    }

    wasm_exec_env_t env = wasm_runtime_create_exec_env(inst, 16 * 1024);
    if (!env) {
        printf("[m1] ERROR: create_exec_env failed\n");
        return NULL;
    }

    size_t heap_after_init = esp_get_free_heap_size();
    printf("[m1] RAM used by runtime+instance: %u bytes\n",
           (unsigned)(heap_at_boot - heap_after_init));

    /* ---- エクスポート関数の取得 ---- */
    wasm_function_inst_t f_init = lookup(inst, "dyne_init");
    wasm_function_inst_t f_step = lookup(inst, "dyne_step");
    wasm_function_inst_t f_get  = lookup(inst, "dyne_get");
    wasm_function_inst_t f_x    = lookup(inst, "dyne_x");
    if (!f_init || !f_step || !f_get || !f_x) {
        return NULL;
    }

    /* ---- dyne_init(n, x0, x1, alpha, dt, bc_left=0, bc_right=0) ---- */
    {
        wasm_val_t args[7], results[1];
        args[0].kind = WASM_I32; args[0].of.i32 = N_POINTS;
        args[1].kind = WASM_F64; args[1].of.f64 = 0.0;
        args[2].kind = WASM_F64; args[2].of.f64 = 1.0;
        args[3].kind = WASM_F64; args[3].of.f64 = ALPHA;
        args[4].kind = WASM_F64; args[4].of.f64 = DT;
        args[5].kind = WASM_I32; args[5].of.i32 = 0; /* Dirichlet */
        args[6].kind = WASM_I32; args[6].of.i32 = 0; /* Dirichlet */
        if (!wasm_runtime_call_wasm_a(env, f_init, 1, results, 7, args)) {
            printf("[m1] ERROR: dyne_init call failed: %s\n",
                   wasm_runtime_get_exception(inst));
            return NULL;
        }
        if (results[0].of.i32 != 0) {
            printf("[m1] ERROR: dyne_init returned %d\n", (int)results[0].of.i32);
            return NULL;
        }
    }
    printf("[m1] init done. running %d steps...\n", STEPS);

    /* ---- 2000 ステップ実行 (時間計測) ---- */
    int64_t t0 = esp_timer_get_time();
    for (int s = 0; s < STEPS; s++) {
        if (!wasm_runtime_call_wasm_a(env, f_step, 0, NULL, 0, NULL)) {
            printf("[m1] ERROR: dyne_step failed at step %d: %s\n",
                   s, wasm_runtime_get_exception(inst));
            return NULL;
        }
        if ((s + 1) % 500 == 0) {
            printf("[m1]   step %d/%d\n", s + 1, STEPS);
        }
    }
    int64_t t1 = esp_timer_get_time();
    double total_ms = (double)(t1 - t0) / 1000.0;
    double per_step_us = (double)(t1 - t0) / (double)STEPS;

    /* ---- 検証: 解析解との誤差 + 代表点の値 ---- */
    double t_final = DT * (double)STEPS;
    double decay = exp(-ALPHA * M_PI * M_PI * t_final);
    double l2 = 0.0, linf = 0.0;
    double u16 = 0.0, u32 = 0.0;
    for (int i = 0; i < N_POINTS; i++) {
        wasm_val_t a[1], r[1];
        a[0].kind = WASM_I32; a[0].of.i32 = i;
        wasm_runtime_call_wasm_a(env, f_get, 1, r, 1, a);
        double u = r[0].of.f64;
        wasm_runtime_call_wasm_a(env, f_x, 1, r, 1, a);
        double x = r[0].of.f64;
        double e = fabs(u - sin(M_PI * x) * decay);
        l2 += e * e;
        if (e > linf) linf = e;
        if (i == 16) u16 = u;
        if (i == 32) u32 = u;
    }
    l2 = sqrt(l2 / (double)N_POINTS);

    /* ---- 限界レポート第2号のサマリ ---- */
    printf("\n==== M1 REPORT (ESP32 / WAMR) ====\n");
    printf("wasm size        : %u bytes\n", (unsigned)wasm_size);
    printf("RAM used (rt+inst): %u bytes\n",
           (unsigned)(heap_at_boot - heap_after_init));
    printf("min free heap    : %u bytes\n",
           (unsigned)esp_get_minimum_free_heap_size());
    printf("steps            : %d\n", STEPS);
    printf("total time       : %.1f ms\n", total_ms);
    printf("per-step time    : %.1f us\n", per_step_us);
    printf("L2  vs analytic  : %.3e  (expect 5.632e-5)\n", l2);
    printf("Linf vs analytic : %.3e  (expect 8.027e-5)\n", linf);
    printf("u[16]            : %.12e (expect 9.816824440839e-2)\n", u16);
    printf("u[32]            : %.12e (expect 1.388308626367e-1)\n", u32);
    printf("==== END ====\n");

    return NULL;
}

void app_main(void)
{
    pthread_t t;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 32 * 1024);
    if (pthread_create(&t, &attr, iwasm_task, NULL) != 0) {
        printf("[m1] ERROR: pthread_create failed\n");
        return;
    }
    pthread_join(t, NULL);
    printf("[m1] done. idle.\n");
}