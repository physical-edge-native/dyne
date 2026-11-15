/*
 * bench-c5: WASMアクターを WAMR (WebAssembly Micro Runtime、マイコン向けの
 * WebAssembly 実行環境) で動かし、速度とメモリを測る単体のファーム。
 *
 * 計測するもの:
 *   (a) mlp-actor (学習済み多層パーセプトロンを実行するアクター、縮小版)
 *       .dynm (計算グラフ+重みの2進ファイル) を読み込み、推論1回あたりの時間、
 *       ヒープの増減、線形メモリ (WebAssembly のメモリ領域) のページ数、
 *       出力の CRC32 を報告する。
 *   (b) heat1d small (1次元熱伝導の格子計算アクター、実機用縮小版)
 *       1秒あたりのステップ数と、温度配列の CRC32 (node-esp32 と同じ定義) を報告する。
 *
 *   (c) 2次元の場のアクターの再生 (形の縮小版 sdf2d small・波動方程式の縮小版 wave2d small)。
 *       dyne-sim2d が記録した1ノード分の呼び出しを再生し、1ステップの時間と CRC の一致を報告する
 *       (2026-11-11 追加、docs/2026-11-11-esp32-interp-bench.md)。
 *
 * 結果は「#BENCH key=value ...」の1行ずつでシリアルに出す。
 * CRC は PC 側 (tools/bench_ref.py expect) と同じ入力・同じ手順で計算した値と
 * ビット一致するはずである (不一致なら実行環境の差による数値の違い = 要調査)。
 *
 * 外部配線なし。起動するたびに全計測を1回行い、終わったら待機する。
 */
#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_clk.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "wasm_export.h"

/* ---- 埋め込み素材 (main/CMakeLists.txt の EMBED_FILES) ---- */
#if CONFIG_BENCH_USE_AOT
extern const uint8_t mlp_bin_start[] asm("_binary_mlp_actor_small_aot_start");
extern const uint8_t mlp_bin_end[] asm("_binary_mlp_actor_small_aot_end");
extern const uint8_t heat_bin_start[] asm("_binary_heat1d_actor_small_aot_start");
extern const uint8_t heat_bin_end[] asm("_binary_heat1d_actor_small_aot_end");
#else
extern const uint8_t mlp_bin_start[] asm("_binary_mlp_actor_small_wasm_start");
extern const uint8_t mlp_bin_end[] asm("_binary_mlp_actor_small_wasm_end");
extern const uint8_t heat_bin_start[] asm("_binary_heat1d_actor_small_wasm_start");
extern const uint8_t heat_bin_end[] asm("_binary_heat1d_actor_small_wasm_end");
#endif
extern const uint8_t ma_start[] asm("_binary_model_a_f64_dynm_start");
extern const uint8_t ma_end[] asm("_binary_model_a_f64_dynm_end");
extern const uint8_t mb_start[] asm("_binary_model_b_f64_dynm_start");
extern const uint8_t mb_end[] asm("_binary_model_b_f64_dynm_end");
extern const uint8_t mb32_start[] asm("_binary_model_b_f32_dynm_start");
extern const uint8_t mb32_end[] asm("_binary_model_b_f32_dynm_end");

/* 入力の組数 (tools/bench_ref.py の ROWS と一致させること) */
#define ROWS 64

/* heat1d の計測条件: (点数, dt, ステップ数)。bench_ref.py の HEAT_CASES と一致させること */
static const struct { int n; double dt; int steps; } HEAT_CASES[] = {
    { 65, 1e-4, 2000 },
    { 1025, 1e-7, 200 },
};

static const char *s_mode = "?";

/* CRC32 (node-esp32 の dyne_crc32 と同じ。zlib.crc32 と同値) */
static uint32_t crc32_bytes(const uint8_t *d, size_t len)
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

static size_t free_int(void) { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); }
static size_t largest_int(void) { return heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); }
static size_t free_psram(void) { return heap_caps_get_free_size(MALLOC_CAP_SPIRAM); }

static void bench_heap(const char *when)
{
    printf("#BENCH bench=heap mode=%s at=%s free_int=%u largest_int=%u min_free_int=%u free_psram=%u\n",
           s_mode, when, (unsigned)free_int(), (unsigned)largest_int(),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           (unsigned)free_psram());
}

/* ---- WAMR の薄い包み ---- */
typedef struct {
    uint8_t *buf;          /* wasm_runtime_load に渡した写し (モジュールの寿命中は保持) */
    wasm_module_t mod;
    wasm_module_inst_t inst;
    wasm_exec_env_t env;
} actor_t;

static bool actor_up(actor_t *a, const uint8_t *bin, size_t len, const char *name)
{
    char err[128];
    memset(a, 0, sizeof(*a));
    size_t f0 = free_int();
    /* 読み込みは書き換え可能なバッファで行う (node-esp32 と同じ。WAMR が内容を書き換える場合がある) */
    a->buf = malloc(len);
    if (!a->buf) { printf("#BENCH bench=error what=malloc_bin actor=%s\n", name); return false; }
    memcpy(a->buf, bin, len);
    int64_t t0 = esp_timer_get_time();
    a->mod = wasm_runtime_load(a->buf, (uint32_t)len, err, sizeof(err));
    int64_t t1 = esp_timer_get_time();
    if (!a->mod) { printf("#BENCH bench=error what=load actor=%s msg=\"%s\"\n", name, err); return false; }
    size_t f1 = free_int();
    /* アプリ用ヒープ 0: アクターは no_std の静的確保で wasm 内 malloc を使わない */
    a->inst = wasm_runtime_instantiate(a->mod, CONFIG_BENCH_EXEC_STACK, 0, err, sizeof(err));
    int64_t t2 = esp_timer_get_time();
    if (!a->inst) { printf("#BENCH bench=error what=instantiate actor=%s msg=\"%s\"\n", name, err); return false; }
    a->env = wasm_runtime_create_exec_env(a->inst, CONFIG_BENCH_EXEC_STACK);
    if (!a->env) { printf("#BENCH bench=error what=exec_env actor=%s\n", name); return false; }
    size_t f2 = free_int();

    wasm_memory_inst_t mem = wasm_runtime_get_default_memory(a->inst);
    uint64_t pages = mem ? wasm_memory_get_cur_page_count(mem) : 0;
    void *base = mem ? wasm_memory_get_base_address(mem) : NULL;
    package_type_t pt = wasm_runtime_get_module_package_type(a->mod);
    printf("#BENCH bench=actor mode=%s actor=%s bin_bytes=%u package=%s load_us=%" PRId64
           " inst_us=%" PRId64 " heap_load=%u heap_inst=%u pages=%u linmem_psram=%d\n",
           s_mode, name, (unsigned)len, pt == Wasm_Module_AoT ? "aot" : "bytecode",
           t1 - t0, t2 - t1, (unsigned)(f0 - f1), (unsigned)(f1 - f2), (unsigned)pages,
           base ? (int)esp_ptr_external_ram(base) : -1);
    return true;
}

static void actor_down(actor_t *a)
{
    if (a->env) wasm_runtime_destroy_exec_env(a->env);
    if (a->inst) wasm_runtime_deinstantiate(a->inst);
    if (a->mod) wasm_runtime_unload(a->mod);
    free(a->buf);
    memset(a, 0, sizeof(*a));
}

static wasm_function_inst_t fn(actor_t *a, const char *name)
{
    wasm_function_inst_t f = wasm_runtime_lookup_function(a->inst, name);
    if (!f) printf("#BENCH bench=error what=lookup fn=%s\n", name);
    return f;
}

/* 引数 0〜1 個 (i32)、戻り値 i32 の呼び出し */
static int32_t call_i(actor_t *a, wasm_function_inst_t f, int nargs, int32_t arg)
{
    wasm_val_t r[1] = { { .kind = WASM_I32 } };
    wasm_val_t v[1] = { { .kind = WASM_I32, .of.i32 = arg } };
    if (!wasm_runtime_call_wasm_a(a->env, f, 1, r, nargs, v)) {
        printf("#BENCH bench=error what=call msg=\"%s\"\n", wasm_runtime_get_exception(a->inst));
        return INT32_MIN;
    }
    return r[0].of.i32;
}

/* アプリ側アドレス (線形メモリ内のオフセット) → ネイティブのポインタ。範囲を検査する */
static void *app_ptr(actor_t *a, uint32_t off, uint32_t size)
{
    if (!wasm_runtime_validate_app_addr(a->inst, off, size)) {
        printf("#BENCH bench=error what=app_addr off=%u size=%u\n", (unsigned)off, (unsigned)size);
        return NULL;
    }
    return wasm_runtime_addr_app_to_native(a->inst, off);
}

/* ---- (a) mlp-actor ---- */
static void bench_mlp(const char *model, const uint8_t *dynm, size_t dlen)
{
    actor_t a;
    size_t h_before = free_int();
    if (!actor_up(&a, mlp_bin_start, (size_t)(mlp_bin_end - mlp_bin_start), "mlp")) { actor_down(&a); return; }
    wasm_function_inst_t f_lbuf = fn(&a, "dyne_load_buf"), f_lcap = fn(&a, "dyne_load_cap"),
                         f_load = fn(&a, "dyne_model_load"), f_ibuf = fn(&a, "dyne_in_buf"),
                         f_obuf = fn(&a, "dyne_out_buf"), f_cap = fn(&a, "dyne_io_cap"),
                         f_nin = fn(&a, "dyne_model_n_in"), f_nout = fn(&a, "dyne_model_n_out"),
                         f_np = fn(&a, "dyne_model_n_params"), f_eval = fn(&a, "dyne_model_eval"),
                         f_batch = fn(&a, "dyne_model_eval_batch");
    if (!(f_lbuf && f_lcap && f_load && f_ibuf && f_obuf && f_cap && f_nin && f_nout && f_np && f_eval && f_batch)) {
        actor_down(&a);
        return;
    }
    /* .dynm を読み込み領域へ書き込んで検証させる */
    uint32_t lcap = (uint32_t)call_i(&a, f_lcap, 0, 0);
    if (dlen > lcap) { printf("#BENCH bench=error what=dynm_too_big len=%u cap=%u\n", (unsigned)dlen, (unsigned)lcap); actor_down(&a); return; }
    uint8_t *lb = app_ptr(&a, (uint32_t)call_i(&a, f_lbuf, 0, 0), (uint32_t)dlen);
    if (!lb) { actor_down(&a); return; }
    memcpy(lb, dynm, dlen);
    int64_t t0 = esp_timer_get_time();
    int32_t rc = call_i(&a, f_load, 1, (int32_t)dlen);
    int64_t t_load = esp_timer_get_time() - t0;
    int n_in = call_i(&a, f_nin, 0, 0), n_out = call_i(&a, f_nout, 0, 0), np = call_i(&a, f_np, 0, 0);
    int io_cap = call_i(&a, f_cap, 0, 0);
    if (rc != 0 || n_in <= 0 || n_out <= 0 || ROWS * n_in > io_cap || ROWS * n_out > io_cap) {
        printf("#BENCH bench=error what=model_load model=%s rc=%d n_in=%d n_out=%d\n", model, (int)rc, n_in, n_out);
        actor_down(&a);
        return;
    }
    double *in = app_ptr(&a, (uint32_t)call_i(&a, f_ibuf, 0, 0), (uint32_t)(8 * ROWS * n_in));
    uint8_t *out = app_ptr(&a, (uint32_t)call_i(&a, f_obuf, 0, 0), (uint32_t)(8 * ROWS * n_out));
    if (!in || !out) { actor_down(&a); return; }

    /* 入力: x[r][i] = (((37r + 11i) mod 201) − 100) / 64  (PC側と同一。丸めなしで表せる値) */
    for (int r = 0; r < ROWS; r++)
        for (int i = 0; i < n_in; i++)
            in[r * n_in + i] = (double)(((37 * r + 11 * i) % 201) - 100) / 64.0;

    /* 照合用: 64組まとめて1回 → 出力 CRC (この時点の出力が PC と比べる対象) */
    if (call_i(&a, f_batch, 1, ROWS) != n_out) { actor_down(&a); return; }
    uint32_t crc = crc32_bytes(out, (size_t)8 * ROWS * n_out);
    double y0;
    memcpy(&y0, out, 8);

    /* 速度 1: 64組まとめた推論の繰り返し (呼び出しの手間を薄めた、計算そのものの速さ) */
    int reps = CONFIG_BENCH_MLP_BATCH_REPS;
    t0 = esp_timer_get_time();
    for (int k = 0; k < reps; k++) call_i(&a, f_batch, 1, ROWS);
    int64_t t_batch = esp_timer_get_time() - t0;
    uint32_t crc2 = crc32_bytes(out, (size_t)8 * ROWS * n_out); /* 入力が同じなので crc と同じはず */

    /* 速度 2: 1組ずつの呼び出し (ホストから毎回呼ぶ実運用に近い形) */
    int calls = CONFIG_BENCH_MLP_SINGLE_CALLS;
    t0 = esp_timer_get_time();
    for (int k = 0; k < calls; k++) call_i(&a, f_eval, 1, n_in);
    int64_t t_single = esp_timer_get_time() - t0;

    /* 積和の回数 (全結合層の 入力×出力 の和) = パラメータ数 − バイアス数 の目安として報告 */
    wasm_memory_inst_t mem = wasm_runtime_get_default_memory(a.inst);
    printf("#BENCH bench=mlp mode=%s model=%s n_in=%d n_out=%d params=%d rows=%d dynm_bytes=%u "
           "load_rc=%d model_load_us=%" PRId64 " us_per_inf_batch=%.3f us_per_inf_single=%.3f "
           "batch_reps=%d single_calls=%d pages=%u heap_used=%u crc=0x%08" PRIX32 " crc_repeat=0x%08" PRIX32
           " y0=%.17g\n",
           s_mode, model, n_in, n_out, np, ROWS, (unsigned)dlen, (int)rc, t_load,
           (double)t_batch / ((double)reps * ROWS), (double)t_single / (double)calls, reps, calls,
           mem ? (unsigned)wasm_memory_get_cur_page_count(mem) : 0u,
           (unsigned)(h_before - free_int()), crc, crc2, y0);
    actor_down(&a);
}

/* ---- (b) heat1d small ---- */
static void bench_heat(int n, double dt, int steps)
{
    actor_t a;
    size_t h_before = free_int();
    if (!actor_up(&a, heat_bin_start, (size_t)(heat_bin_end - heat_bin_start), "heat1d")) { actor_down(&a); return; }
    wasm_function_inst_t f_init = fn(&a, "dyne_init"), f_step = fn(&a, "dyne_step"), f_get = fn(&a, "dyne_get");
    if (!(f_init && f_step && f_get)) { actor_down(&a); return; }
    /* 単一ノード・区間 [0,1]・alpha=1・両端 u=0 固定 (境界条件コード 0) */
    wasm_val_t ia[7], ir[1] = { { .kind = WASM_I32 } };
    ia[0].kind = WASM_I32; ia[0].of.i32 = n;
    ia[1].kind = WASM_F64; ia[1].of.f64 = 0.0;
    ia[2].kind = WASM_F64; ia[2].of.f64 = 1.0;
    ia[3].kind = WASM_F64; ia[3].of.f64 = 1.0;
    ia[4].kind = WASM_F64; ia[4].of.f64 = dt;
    ia[5].kind = WASM_I32; ia[5].of.i32 = 0;
    ia[6].kind = WASM_I32; ia[6].of.i32 = 0;
    if (!wasm_runtime_call_wasm_a(a.env, f_init, 1, ir, 7, ia) || ir[0].of.i32 != 0) {
        printf("#BENCH bench=error what=heat_init n=%d rc=%d\n", n, (int)ir[0].of.i32);
        actor_down(&a);
        return;
    }
    int64_t t0 = esp_timer_get_time();
    for (int s = 0; s < steps; s++) wasm_runtime_call_wasm_a(a.env, f_step, 0, NULL, 0, NULL);
    int64_t t = esp_timer_get_time() - t0;

    /* 温度配列の f64 バイト列の CRC32 (node-esp32 の BSP done の crc と同じ定義) */
    uint8_t *all = malloc((size_t)n * 8);
    uint32_t crc = 0;
    if (all) {
        for (int i = 0; i < n; i++) {
            wasm_val_t ga[1] = { { .kind = WASM_I32, .of.i32 = i } }, gr[1] = { { .kind = WASM_F64 } };
            wasm_runtime_call_wasm_a(a.env, f_get, 1, gr, 1, ga);
            memcpy(all + (size_t)i * 8, &gr[0].of.f64, 8);
        }
        crc = crc32_bytes(all, (size_t)n * 8);
        free(all);
    }
    wasm_memory_inst_t mem = wasm_runtime_get_default_memory(a.inst);
    printf("#BENCH bench=heat1d mode=%s n=%d dt=%g steps=%d total_us=%" PRId64 " steps_per_s=%.1f "
           "ns_per_point=%.1f pages=%u heap_used=%u crc=0x%08" PRIX32 "\n",
           s_mode, n, dt, steps, t, (double)steps * 1e6 / (double)t,
           (double)t * 1e3 / ((double)steps * n),
           mem ? (unsigned)wasm_memory_get_cur_page_count(mem) : 0u,
           (unsigned)(h_before - free_int()), crc);
    actor_down(&a);
}

/* ---- (c) 2次元の場のアクターの再生 (2026-11-11、docs/2026-11-11-esp32-interp-bench.md) ----
 * dyne-sim2d --dump-node N --dump-file X.rec が記録した、ノード N のアクターへの呼び出し
 * (設定・面・初期化・受信メールボックス・送信メールボックス・ステップ) をそのまま再生する。
 * 実機の1ノードが分割計算の中で行う処理と同じ (通信の待ちだけを除く)。最後に区画の
 * φ (と d・w) の CRC32 を、記録の末尾にあるシミュレータの値と照合する。
 * 記録の書式は crates/dyne-sim2d/src/main.rs の REC_* と一致させること。 */
/* 形のアクターは、速くする工夫の組み合わせ (docs/2026-11-11-shape-speedup.md) ごとに wasm と記録を持つ。
 * 名前の対応は main/CMakeLists.txt と tools/interp_bench.py の SHAPE_SETS に合わせること */
#define ASSET(sym) extern const uint8_t sym##_start[] asm("_binary_" #sym "_start"); \
                   extern const uint8_t sym##_end[] asm("_binary_" #sym "_end");
ASSET(sdf2d_grid_actor_small_wasm)  /* 推奨: 結果を変えない工夫 + 標準ライブラリの sqrt (std-math) */
ASSET(sdf2d_base_small_wasm)        /* 比較用: 工夫の前 (2026-11-11 夕方に ESPr で測ったもの) */
ASSET(sdf2d_nostd_small_wasm)       /* 結果を変えない工夫だけ (libm の sqrt のまま) */
ASSET(sdf2d_f32_small_wasm)         /* 単精度 (f32) + std-math */
ASSET(wave2d_actor_small_wasm)
ASSET(shape_n13_rec)
ASSET(shape_n5_rec)
ASSET(shape_sw2_n5_rec)
ASSET(shape_f32_n5_rec)
ASSET(shape_h5_n5_rec)
ASSET(shape_p4h_rec)               /* PAN4H (首振り7台) + 視野の中だけの減衰、最も重いノード */
ASSET(wave_n5_rec)
#define BIN(sym) sym##_start, (size_t)(sym##_end - sym##_start)

static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }
static double rdf(const uint8_t *p) { double v; memcpy(&v, p, 8); return v; }

typedef struct { int64_t n, sum, min, max; } stat_t;
static void st_add(stat_t *s, int64_t v)
{
    if (s->n == 0 || v < s->min) s->min = v;
    if (s->n == 0 || v > s->max) s->max = v;
    s->sum += v;
    s->n++;
}
static double st_mean(const stat_t *s) { return s->n ? (double)s->sum / (double)s->n : 0.0; }

static bool call_v(actor_t *a, wasm_function_inst_t f, uint32_t nres, wasm_val_t *res, uint32_t nargs, wasm_val_t *args)
{
    if (!wasm_runtime_call_wasm_a(a->env, f, nres, res, nargs, args)) {
        printf("#BENCH bench=error what=call msg=\"%s\"\n", wasm_runtime_get_exception(a->inst));
        return false;
    }
    return true;
}

#define I32V(x) ((wasm_val_t){ .kind = WASM_I32, .of.i32 = (int32_t)(x) })
#define I64V(x) ((wasm_val_t){ .kind = WASM_I64, .of.i64 = (int64_t)(x) })
#define F64V(x) ((wasm_val_t){ .kind = WASM_F64, .of.f64 = (x) })

static void bench_replay(const char *item, const char *opts, const uint8_t *bin, size_t blen, const uint8_t *rec, size_t rlen)
{
    actor_t a;
    size_t h_before = free_int();
    if (!actor_up(&a, bin, blen, item)) { actor_down(&a); return; }
    if (rlen < 8 || memcmp(rec, "DYNEREC1", 8) != 0) {
        printf("#BENCH bench=error what=rec_header item=%s\n", item);
        actor_down(&a);
        return;
    }
    wasm_function_inst_t f_cfg = wasm_runtime_lookup_function(a.inst, "dyne_cfg");
    wasm_function_inst_t f_face = fn(&a, "dyne_set_face"), f_init = fn(&a, "dyne_init2d"),
                         f_mbuf = fn(&a, "dyne_msg_buf"), f_mcap = fn(&a, "dyne_msg_cap"),
                         f_in = fn(&a, "dyne_inbox_write"), f_out = fn(&a, "dyne_outbox_read"),
                         f_step = fn(&a, "dyne_step"), f_get = fn(&a, "dyne_get"), f_len = fn(&a, "dyne_len");
    wasm_function_inst_t f_aux = wasm_runtime_lookup_function(a.inst, "dyne_get_aux");
    wasm_function_inst_t f_sweep = wasm_runtime_lookup_function(a.inst, "dyne_bench_sweep");
    wasm_function_inst_t f_pend = wasm_runtime_lookup_function(a.inst, "dyne_pending");
    wasm_function_inst_t f_ovf = wasm_runtime_lookup_function(a.inst, "dyne_overflow");
    wasm_function_inst_t f_part = wasm_runtime_lookup_function(a.inst, "dyne_step_part");
    if (!(f_face && f_init && f_mbuf && f_mcap && f_in && f_out && f_step && f_get && f_len)) { actor_down(&a); return; }
    uint32_t mbuf = (uint32_t)call_i(&a, f_mbuf, 0, 0), mcap = (uint32_t)call_i(&a, f_mcap, 0, 0);

    stat_t st_step = { 0 }, st_total = { 0 }, st_in = { 0 }, st_part[3] = { { 0 } };
    int64_t cur_in = 0, cur_out = 0;
    uint32_t steps = 0, n_in = 0, max_pend = 0, errors = 0;
    uint64_t in_bytes = 0;
    uint32_t nx = 0, ny = 0, exp_n = 0, exp_crc[3] = { 0 };
    bool end_seen = false;
    size_t p = 8;
    while (p < rlen && !end_seen && errors == 0) {
        uint8_t tag = rec[p++];
        switch (tag) {
        case 1: { /* CFG */
            wasm_val_t v[2] = { I32V(rd32(rec + p)), F64V(rdf(rec + p + 4)) };
            p += 12;
            if (!f_cfg || !call_v(&a, f_cfg, 0, NULL, 2, v)) errors++;
            break;
        }
        case 2: { /* FACE */
            wasm_val_t v[3] = { I32V(rd32(rec + p)), I32V(rd32(rec + p + 4)), I64V(rd64(rec + p + 8)) };
            p += 16;
            if (!call_v(&a, f_face, 0, NULL, 3, v)) errors++;
            break;
        }
        case 3: { /* INIT */
            nx = rd32(rec + p);
            ny = rd32(rec + p + 4);
            wasm_val_t v[7] = { I32V(nx), I32V(ny), I32V(rd32(rec + p + 8)), I32V(rd32(rec + p + 12)),
                                F64V(rdf(rec + p + 16)), F64V(rdf(rec + p + 24)), F64V(rdf(rec + p + 32)) };
            wasm_val_t r[1] = { { .kind = WASM_I32 } };
            p += 40;
            if (!call_v(&a, f_init, 1, r, 7, v) || r[0].of.i32 != 0) {
                printf("#BENCH bench=error what=init2d item=%s rc=%d\n", item, (int)r[0].of.i32);
                errors++;
            }
            break;
        }
        case 4: { /* INBOX */
            uint32_t len = rd32(rec + p);
            p += 4;
            uint8_t *dst = len <= mcap ? app_ptr(&a, mbuf, len) : NULL;
            if (!dst) { printf("#BENCH bench=error what=inbox_len item=%s len=%u cap=%u\n", item, (unsigned)len, (unsigned)mcap); errors++; break; }
            int64_t t0 = esp_timer_get_time();
            memcpy(dst, rec + p, len);
            wasm_val_t v[2] = { I32V(mbuf), I32V(len) };
            wasm_val_t r[1] = { { .kind = WASM_I32 } };
            bool ok = call_v(&a, f_in, 1, r, 2, v);
            int64_t dt = esp_timer_get_time() - t0;
            cur_in += dt;
            st_add(&st_in, dt);
            n_in++;
            in_bytes += len;
            if (!ok || r[0].of.i32 != 0) { printf("#BENCH bench=error what=inbox item=%s rc=%d\n", item, (int)r[0].of.i32); errors++; }
            p += len;
            break;
        }
        case 5: { /* OUTBOX */
            int64_t t0 = esp_timer_get_time();
            wasm_val_t v[2] = { I32V(mbuf), I32V(mcap) };
            wasm_val_t r[1] = { { .kind = WASM_I32 } };
            if (!call_v(&a, f_out, 1, r, 2, v)) errors++;
            cur_out += esp_timer_get_time() - t0;
            break;
        }
        case 6: { /* STEP */
            int64_t t0 = esp_timer_get_time();
            wasm_val_t r[1] = { { .kind = WASM_I32 } };
            bool ok;
            if (f_part) {
                /* 内訳を測る: dyne_step_part(0/1/2) を順に呼ぶ (dyne_step と同じ結果。
                 * 0 = 界面の確認と距離の反復、1 = 光線の取り込み、2 = 減衰・統合) */
                ok = true;
                int32_t rc0 = 0;
                for (int k = 0; k < 3 && ok; k++) {
                    int64_t tp = esp_timer_get_time();
                    wasm_val_t v[1] = { I32V(k) };
                    ok = call_v(&a, f_part, 1, r, 1, v);
                    st_add(&st_part[k], esp_timer_get_time() - tp);
                    if (k == 0) rc0 = r[0].of.i32;
                }
                r[0].of.i32 = rc0;
            } else {
                ok = call_v(&a, f_step, 1, r, 0, NULL);
            }
            int64_t dt = esp_timer_get_time() - t0;
            if (!ok || r[0].of.i32 != 0) { printf("#BENCH bench=error what=step item=%s rc=%d\n", item, (int)r[0].of.i32); errors++; }
            st_add(&st_step, dt);
            st_add(&st_total, dt + cur_in + cur_out);
            cur_in = cur_out = 0;
            steps++;
            if (f_pend) {
                uint32_t np = (uint32_t)call_i(&a, f_pend, 0, 0);
                if (np > max_pend) max_pend = np;
            }
            break;
        }
        case 7: /* END */
            exp_n = rd32(rec + p);
            for (int k = 0; k < 3; k++) exp_crc[k] = rd32(rec + p + 4 + 4 * k);
            p += 16;
            end_seen = true;
            break;
        default:
            printf("#BENCH bench=error what=rec_tag item=%s tag=%u at=%u\n", item, tag, (unsigned)(p - 1));
            errors++;
        }
    }
    size_t heap_used = h_before - free_int();
    size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    /* 照合: 区画の φ (と d・w) の f64 バイト列の CRC32 (dyne-sim2d の field_crc と同じ定義) */
    uint32_t got_n = (uint32_t)call_i(&a, f_len, 0, 0), got_crc[3] = { 0 };
    uint8_t *buf = malloc((size_t)got_n * 8);
    int nq = f_aux ? 3 : 1;
    if (buf) {
        for (int q = 0; q < nq; q++) {
            for (uint32_t k = 0; k < got_n; k++) {
                wasm_val_t r[1] = { { .kind = WASM_F64 } };
                if (q == 0) {
                    wasm_val_t v[1] = { I32V(k) };
                    call_v(&a, f_get, 1, r, 1, v);
                } else {
                    wasm_val_t v[2] = { I32V(q), I32V(k) }; /* AUX_D = 1, AUX_W = 2 */
                    call_v(&a, f_aux, 1, r, 2, v);
                }
                memcpy(buf + (size_t)k * 8, &r[0].of.f64, 8);
            }
            got_crc[q] = crc32_bytes(buf, (size_t)got_n * 8);
        }
        free(buf);
    }
    bool match = end_seen && errors == 0 && got_n == exp_n && got_crc[0] == exp_crc[0] &&
                 (nq == 1 || (got_crc[1] == exp_crc[1] && got_crc[2] == exp_crc[2]));
    int64_t ovf = 0;
    if (f_ovf) {
        wasm_val_t r[1] = { { .kind = WASM_I64 } };
        if (call_v(&a, f_ovf, 1, r, 0, NULL)) ovf = r[0].of.i64;
    }
    wasm_memory_inst_t mem = wasm_runtime_get_default_memory(a.inst);
    printf("#BENCH bench=replay mode=%s item=%s opts=%s nx=%u ny=%u steps=%u step_ms_min=%.3f step_ms_mean=%.3f step_ms_max=%.3f "
           "total_ms_min=%.3f total_ms_mean=%.3f total_ms_max=%.3f inbox_calls=%u inbox_ms_mean=%.3f inbox_ms_max=%.3f "
           "inbox_bytes=%" PRIu64 " max_pending=%u overflow=%" PRId64 " pages=%u heap_used=%u min_free_int=%u "
           "crc=%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32 " expect=%08" PRIx32 "/%08" PRIx32 "/%08" PRIx32 " match=%d\n",
           s_mode, item, opts, (unsigned)nx, (unsigned)ny, (unsigned)steps, st_step.min / 1e3, st_mean(&st_step) / 1e3,
           st_step.max / 1e3, st_total.min / 1e3, st_mean(&st_total) / 1e3, st_total.max / 1e3, (unsigned)n_in,
           st_mean(&st_in) / 1e3, st_in.max / 1e3, in_bytes, (unsigned)max_pend, ovf,
           mem ? (unsigned)wasm_memory_get_cur_page_count(mem) : 0u, (unsigned)heap_used, (unsigned)min_free,
           got_crc[0], got_crc[1], got_crc[2], exp_crc[0], exp_crc[1], exp_crc[2], (int)match);
    if (f_part) {
        /* 1ステップの内訳 (dyne_step_part。2026-11-11、docs/2026-11-11-shape-speedup.md) */
        printf("#BENCH bench=parts mode=%s item=%s opts=%s sweep_ms_mean=%.3f intake_ms_mean=%.3f fuse_ms_mean=%.3f "
               "sweep_ms_max=%.3f intake_ms_max=%.3f fuse_ms_max=%.3f\n",
               s_mode, item, opts, st_mean(&st_part[0]) / 1e3, st_mean(&st_part[1]) / 1e3, st_mean(&st_part[2]) / 1e3,
               st_part[0].max / 1e3, st_part[1].max / 1e3, st_part[2].max / 1e3);
    }

    /* 距離の反復だけの時間 (照合の後なので状態を書き換えてよい)。1ステップの時間からこれを
     * 引いたものが光線の処理 (取り込み・中継の判定・減衰・統合) の時間 */
    if (f_sweep) {
        const int n = 20;
        int64_t t0 = esp_timer_get_time();
        call_i(&a, f_sweep, 1, n);
        int64_t t = esp_timer_get_time() - t0;
        printf("#BENCH bench=sweep mode=%s item=%s opts=%s nx=%u ny=%u sweeps=%d sweep_ms=%.3f ns_per_cell=%.1f\n",
               s_mode, item, opts, (unsigned)nx, (unsigned)ny, n, (double)t / 1e3 / n,
               (double)t * 1e3 / ((double)n * nx * ny));
    }
    actor_down(&a);
}

/* WAMR の ESP-IDF 移植は pthread_self() を使うため、FreeRTOS のタスク(app_main)から直接呼ぶと
 * "pthread_self ... Failed to find current thread ID!" で止まる (2026-11-11 ESPr で発生)。
 * node-esp32 と同じく、本体を pthread の中で動かす。 */
static void *bench_main(void *arg);

void app_main(void)
{
    pthread_t t;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 32 * 1024); /* node-esp32 と同じ */
    if (pthread_create(&t, &attr, bench_main, NULL) != 0) {
        printf("#BENCH bench=error what=pthread_create\n");
        return;
    }
    pthread_join(t, NULL);
}

static void *bench_main(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500)); /* USB シリアルの接続待ち (最初の行を取りこぼさないため) */
#if CONFIG_BENCH_USE_AOT
    s_mode = "aot";
#elif CONFIG_WAMR_INTERP_FAST
    s_mode = "fast";
#else
    s_mode = "classic";
#endif
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    uint32_t ver[3];
    wasm_runtime_get_version(&ver[0], &ver[1], &ver[2]);
    printf("#BENCH bench=start mode=%s target=%s rev=%d cpu_mhz=%d idf=%s wamr=%u.%u.%u psram_total=%u\n",
           s_mode, CONFIG_IDF_TARGET, ci.revision, esp_clk_cpu_freq() / 1000000, esp_get_idf_version(),
           (unsigned)ver[0], (unsigned)ver[1], (unsigned)ver[2],
           (unsigned)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    bench_heap("boot");

    RuntimeInitArgs ia;
    memset(&ia, 0, sizeof(ia));
    ia.mem_alloc_type = Alloc_With_System_Allocator; /* node-esp32 と同じ */
    if (!wasm_runtime_full_init(&ia)) {
        printf("#BENCH bench=error what=runtime_init\n");
        return NULL;
    }
    bench_heap("runtime_init");

    /* 2次元の場のアクター (形の縮小版・波動方程式の縮小版) の再生。判定に要るので先に測る。
     * 形は、速くする工夫の組み合わせ (opts) ごとに測る (docs/2026-11-11-shape-speedup.md §6)。
     * opts の意味: exact = 結果を変えない工夫 (光線の範囲の絞り込み・floor の置き換え)、
     *   std = 標準ライブラリの sqrt (結果は同じ)、libm = libm の sqrt のまま、base = 工夫の前、
     *   sw2 = 距離の反復を2ステップに1回、f32 = 単精度、h5 = 格子の刻み 5cm (区画 18×14点)、
     *   inview = 減衰・忘却を視野の中だけに効かせる (shape_p4h は首振り PAN4H の場面 room6_move_pan4h)。
     * 先頭の2つ (推奨の exact+std、ノード13・5) は工夫の前と同じ記録・同じ CRC で照合する */
    const struct { const char *item, *opts; const uint8_t *bin; size_t blen; const uint8_t *rec; size_t rlen; } items[] = {
        { "shape_n13", "exact+std", BIN(sdf2d_grid_actor_small_wasm), BIN(shape_n13_rec) },
        { "shape_n5", "exact+std", BIN(sdf2d_grid_actor_small_wasm), BIN(shape_n5_rec) },
        { "shape_n5", "base", BIN(sdf2d_base_small_wasm), BIN(shape_n5_rec) },
        { "shape_n5", "exact+libm", BIN(sdf2d_nostd_small_wasm), BIN(shape_n5_rec) },
        { "shape_n5", "exact+std+sw2", BIN(sdf2d_grid_actor_small_wasm), BIN(shape_sw2_n5_rec) },
        { "shape_n5", "exact+std+f32", BIN(sdf2d_f32_small_wasm), BIN(shape_f32_n5_rec) },
        { "shape_n5", "exact+std+h5", BIN(sdf2d_grid_actor_small_wasm), BIN(shape_h5_n5_rec) },
        { "shape_p4h", "exact+std+inview", BIN(sdf2d_grid_actor_small_wasm), BIN(shape_p4h_rec) },
    };
    for (size_t k = 0; k < sizeof(items) / sizeof(items[0]); k++) {
        bench_replay(items[k].item, items[k].opts, items[k].bin, items[k].blen, items[k].rec, items[k].rlen);
        if (k == 0) bench_heap("after_shape_n13");
    }
    bench_replay("wave_n5", "-", BIN(wave2d_actor_small_wasm), BIN(wave_n5_rec));
    bench_heap("after_2d");
#if CONFIG_BENCH_RUN_LEGACY
    bench_mlp("model_a_f64", ma_start, (size_t)(ma_end - ma_start));
    bench_mlp("model_b_f64", mb_start, (size_t)(mb_end - mb_start));
    bench_mlp("model_b_f32", mb32_start, (size_t)(mb32_end - mb32_start));
    for (size_t k = 0; k < sizeof(HEAT_CASES) / sizeof(HEAT_CASES[0]); k++)
        bench_heat(HEAT_CASES[k].n, HEAT_CASES[k].dt, HEAT_CASES[k].steps);
#endif

    bench_heap("end");
    printf("#BENCH bench=done mode=%s\n", s_mode);
    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
    return NULL;
}
