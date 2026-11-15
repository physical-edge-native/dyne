/*
 * 場のアクター (メールボックス方式) をワイヤ形式v2で同期させる経路 (2026-11-08)
 *
 * 選択: 積み込んだ wasm が送信/受信メールボックスの差込口 (dyne_outbox_read /
 * dyne_inbox_write ほか、crates/heat2d-actor・sdf2d-grid-actor と同じ一式) を持つとき、
 * wasm_up() がこの経路を有効にする。持たない wasm (現行の heat1d_actor_small) では
 * 本ファイルの関数は呼ばれず、既存の1次元の経路 (40バイトの実機形式) は無変化。
 *
 * データ面: 1データグラム = 外箱 (dyne_wire2.h。通番+CRC32) に包んだ
 * 「1つの界面・1つのステップぶんの枠つき v2 メッセージ列」。ファームは中身を
 * 変換しない (送信メールボックスから出たバイト列を界面ごとに分けて包むだけ、
 * 受信側は外箱を外して受信メールボックスへそのまま渡す)。
 * 同期の組み立ては既存の run_bsp と同じ: 送信控え (直近2ステップ)・先着分の取り置き・
 * 過去分を受けたら控えを再送で応答・返事待ちの上限で再送 (バックオフつき)・終了時ドレイン。
 * シミュレータ (crates/dyne-sim2d) の配送規則とも同じ。
 *
 * 設定注入 (dyne/cfg/<id>、本経路の時のみ。暫定の形。docs/2026-11-08-firmware-wire-v2.md):
 *   { "gen":G, "cluster":{"steps":S,"alpha":a,"dt":dt,"step_period_ms":p},
 *     "node":{"idx":i},
 *     "field2d":{"nx":..,"ny":..,"ix0":..,"iy0":..,"h":..,
 *                "faces":[{"face":"w|e|s|n","key":"interface|dirichlet|zero_grad",
 *                          "iface":id,"value":v}, ...],
 *                "cfg":[[key,val], ...]},          ← アクターの dyne_cfg (任意)
 *     "ifaces":[{"id":id,"peer":"ip:port"}, ...],
 *     "mode":{"field_every":k,"rt_every":k},
 *     "feed":{"range":false,"imu":false} }   ← 2026-11-13: センサの値をアクターへ渡すか (任意)
 *   2026-11-13 追加: mode.shape_every (形の区画のメッセージを送る間隔 [ステップ]。形のアクター
 *   (dyne_get_aux を持つ) なら既定 15、それ以外は 0 = 送らない)、field2d.gnx / gny (全体の格子の点の数。任意)
 *   2026-11-13 追加 (IMU の融合。docs/2026-11-13-firmware-nodes.md §13):
 *     "imu":{"t0_us":T, "start_delay_ms":3000, "grid_ms":10000, "time_mode":"stamp|arrival",
 *            "offset_us":0, "offset_per_idx_us":0}
 *   feed.imu のとき: ステップ0を同期した時刻 t0 まで待ってから始め、ステップ s を t0 + s·dt に合わせて進める
 *   (step_period_ms は dt と同じ。省けば dt から決める)。t0 は imu.t0_us → field2d.cfg のキー2 → 自動
 *   (今 + start_delay_ms 以降で最初の grid_ms の倍数) の順で決め、アクターの CFG_T0_US に入れる。
 *   time_mode・offset_us + offset_per_idx_us × node.idx は CFG_TIME_MODE・CFG_TS_OFFSET_US に入れる (わざと同期を崩す)。
 *   融合のアクター (dyne_imu_push を持つ) なら、区間の番号 (dyne_get(0)) が変わるたびに
 *   dyne/node/<id>/imu_fused へ {"step":s,"v":[dyne_get(0..)]} を発行する。
 *   2026-11-15 追加 (実機の入力の再生の記録。GitHub issue #2、docs/2026-11-15-hw-replay.md):
 *     "replay":{"log":true}   ← 任意。既定 false。実行ごとに PC が決める (同じファームで記録する・しないを選べる)
 *   true のとき、actor_setup・range_fed・imu_fed を QoS 1 で発行する (下の「再生の記録」)。
 *   false のときは range_fed も発行しない (2026-11-15 から。以前は feed.range なら常に QoS 0 で発行していた)。
 *
 * 2026-11-13 (docs/2026-11-13-firmware-nodes.md):
 *  - feed.range: 測距 (dyne_range.c) の最新のフレームを、各ステップの送信の後・受信待ちの前に
 *    RANGE_FRAME (src = センサID) として受信メールボックスへ入れる (dyne-sim2d と同じ位置)。
 *    1ステップに渡すのは最新の1枚 (それより前の未渡しのフレームは飛ばして数える)。
 *    渡したフレームは dyne/node/<id>/range_fed に発行する (PC で同じ入力を再生できるように。
 *    2026-11-15 から replay.log のときだけ)。
 *    初期化では、PC の field2d.cfg の後に、形のアクターの CFG_ZONE_ROW (行を取り出して送るなら 0) と
 *    センサの一覧 (CFG_SENSOR_SEL/X/Y/YAW。首振りなら位置ごとに別のID) を注入する。
 *  - feed.imu: IMU (dyne_imu.c) の標本を dyne_step の前に dyne_imu_push で入れる
 *    (crates/imu-fusion-actor の差込口)。アクターが dyne_imu_need_until_us で時刻を示したら、
 *    その時刻までの標本が揃うのを最大 V2_IMU_WAIT_MS 待つ。
 *
 * 分割送信は無し (1436バイトを超える配送単位は送らず誤りで停止する)。実機用の縮小版のアクターは
 * 構造上この上限に収まる (docs/2026-11-13-firmware-nodes.md §G10)。
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "cJSON.h"

#include "wasm_export.h"
#include "dyne_node.h"
#include "dyne_wire2.h"
#include "node_v2.h"
#include "dyne_timesync.h" /* 2026-11-11: 時刻印 */
#include "dyne_sense.h"    /* 2026-11-13: 測距・IMU の値をアクターへ */
#include "dyne_shape_msg.h" /* 2026-11-13: 形の区画のメッセージ (PC の常駐版へ) */

static const char *TAG = "dyne-v2";

#define V2_MAX_IFACES  4      /* 2次元: 東西南北 */
#define V2_TIMEOUT_US  100000 /* 返事待ちの上限 (1次元の経路と同じ) */
#define V2_RESEND_MIN_US 20000
#define V2_RECV_SLICE_MS 20
#define V2_DRAIN_MS    3000

/* 面ID・面の種類・戻り値 (crates/dyne-wire/src/abi.rs の field2d と同じ値) */
#define F2_BC_DIRICHLET 0
#define F2_BC_INTERFACE 1
#define F2_BC_ZERO_GRAD 2
#define F2_INBOX_OK     0
#define F2_STEP_OK      0

/* 形の格子型アクターの dyne_cfg のキー (crates/dyne-wire/src/abi.rs の sdf2d と同じ値) */
#define SDF_CFG_ZONE_ROW   9
#define SDF_CFG_SENSOR_SEL 10
#define SDF_CFG_SENSOR_X   11
#define SDF_CFG_SENSOR_Y   12
#define SDF_CFG_SENSOR_YAW 13
#define V2_IMU_WAIT_MS     300  /* IMU の標本が揃うのを待つ上限 */
#define V2_IMU_BATCH       32
#define SDF_CFG_ORIGIN_X   0
#define SDF_CFG_ORIGIN_Y   1
#define SDF_AUX_D          1
#define SDF_AUX_W          2
#define V2_SHAPE_DEFAULT_EVERY 15     /* 15Hz で約1秒ごと */
/* IMU の融合のアクターの dyne_cfg のキー (crates/dyne-wire/src/abi.rs の imu と同じ値) */
#define IMU_CFG_TIME_MODE    1
#define IMU_CFG_T0_US        2
#define IMU_CFG_TS_OFFSET_US 3
#define IMU_FUSED_MAX        32
#define V2_SHAPE_MAX_OUTBOX    32768  /* MQTT のアウトボックスがこれを超えていたら送らない */

/* ==== 差込口 ==== */
static bool s_active;
static wasm_module_inst_t s_inst;
static wasm_exec_env_t s_env;
static wasm_function_inst_t f_set_face, f_init2d, f_msg_buf, f_msg_cap, f_outbox, f_inbox,
    f_step, f_len, f_get, f_cfg, f_dropped, f_rejected, f_missing;
static wasm_function_inst_t f_imu_push, f_imu_need; /* 2026-11-13: IMU の融合のアクター (任意) */
static wasm_function_inst_t f_get_aux; /* 2026-11-13: 形のアクター (d・w の読み出し。任意) */

static bool alloc_cfg(void);

bool dyne_v2_bind(wasm_module_inst_t inst, wasm_exec_env_t env)
{
    s_inst = inst;
    s_env = env;
    f_outbox = wasm_runtime_lookup_function(inst, "dyne_outbox_read");
    f_inbox = wasm_runtime_lookup_function(inst, "dyne_inbox_write");
    if (!f_outbox || !f_inbox) return false; /* メールボックス方式でない → 既存の経路 */
    f_set_face = wasm_runtime_lookup_function(inst, "dyne_set_face");
    f_init2d = wasm_runtime_lookup_function(inst, "dyne_init2d");
    f_msg_buf = wasm_runtime_lookup_function(inst, "dyne_msg_buf");
    f_msg_cap = wasm_runtime_lookup_function(inst, "dyne_msg_cap");
    f_step = wasm_runtime_lookup_function(inst, "dyne_step");
    f_len = wasm_runtime_lookup_function(inst, "dyne_len");
    f_get = wasm_runtime_lookup_function(inst, "dyne_get");
    f_cfg = wasm_runtime_lookup_function(inst, "dyne_cfg");
    f_dropped = wasm_runtime_lookup_function(inst, "dyne_dropped");
    f_rejected = wasm_runtime_lookup_function(inst, "dyne_rejected");
    f_missing = wasm_runtime_lookup_function(inst, "dyne_missing");
    f_imu_push = wasm_runtime_lookup_function(inst, "dyne_imu_push");
    f_imu_need = wasm_runtime_lookup_function(inst, "dyne_imu_need_until_us");
    f_get_aux = wasm_runtime_lookup_function(inst, "dyne_get_aux");
    s_active = f_set_face && f_init2d && f_msg_buf && f_msg_cap && f_step && f_len && f_get;
    if (s_active && !alloc_cfg()) { ESP_LOGE(TAG, "v2 cfg alloc"); s_active = false; }
    if (!s_active) ESP_LOGE(TAG, "mailbox exports incomplete");
    else ESP_LOGI(TAG, "mailbox actor detected: wire v2 path");
    return s_active;
}

bool dyne_v2_active(void)
{
    return s_active;
}

static uint32_t call_u32(wasm_function_inst_t f)
{
    wasm_val_t r[1];
    wasm_runtime_call_wasm_a(s_env, f, 1, r, 0, NULL);
    return (uint32_t)r[0].of.i32;
}

static uint64_t call_u64(wasm_function_inst_t f)
{
    if (!f) return 0;
    wasm_val_t r[1];
    wasm_runtime_call_wasm_a(s_env, f, 1, r, 0, NULL);
    return (uint64_t)r[0].of.i64;
}

static double get_k(uint32_t k)
{
    wasm_val_t a[1] = { { .kind = WASM_I32, .of.i32 = (int32_t)k } };
    wasm_val_t r[1];
    wasm_runtime_call_wasm_a(s_env, f_get, 1, r, 1, a);
    return r[0].of.f64;
}

/* ==== 設定 ==== */
#define V2_MAX_ACTOR_CFG 32
/* 本経路の設定。静的領域に置くと既存の1次元の経路の空きメモリを減らすので、
 * 本経路が有効になったとき (dyne_v2_bind) にだけヒープに確保する */
typedef struct {
    uint32_t nx, ny, ix0, iy0;
    double h, alpha, dt;
    uint32_t steps;
    int step_period_ms;
    int face_kind[4];
    uint64_t face_param[4];
    int n_cfg;
    uint32_t cfg_key[V2_MAX_ACTOR_CFG];
    double cfg_val[V2_MAX_ACTOR_CFG];
    int n_ifaces;
    struct { uint32_t id; struct sockaddr_in peer; } ifaces[V2_MAX_IFACES];
    int field_every, rt_every;
    bool feed_range, feed_imu; /* 2026-11-13 */
    int shape_every;          /* 2026-11-13: 形の区画のメッセージの間隔 (0 = 送らない) */
    uint32_t gnx, gny;
    /* 2026-11-13: IMU の融合の実行 */
    bool imu_t0_given;
    int64_t imu_t0;
    int imu_start_delay_ms, imu_grid_ms;
    int imu_time_mode;        /* -1 = 注入しない */
    bool imu_off_given;
    double imu_off_us;
    bool replay_log;          /* 2026-11-15: 再生の記録 (replay.log) */
} v2cfg_t;
static v2cfg_t *s_cp;
#define s_c (*s_cp)

static bool alloc_cfg(void)
{
    if (!s_cp) s_cp = calloc(1, sizeof(v2cfg_t));
    return s_cp != NULL;
}

static int num_i(const cJSON *o, const char *k, int def)
{
    const cJSON *v = cJSON_GetObjectItem(o, k);
    return cJSON_IsNumber(v) ? v->valueint : def;
}
static double num_d(const cJSON *o, const char *k, double def)
{
    const cJSON *v = cJSON_GetObjectItem(o, k);
    return cJSON_IsNumber(v) ? v->valuedouble : def;
}

static bool parse_cfg_v2(const char *payload, int len, uint32_t *gen, int *idx,
                         char *err, size_t errlen)
{
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) { snprintf(err, errlen, "json parse"); return false; }
    bool ok = false;
    memset(&s_c, 0, sizeof(s_c));
    do {
        const cJSON *cl = cJSON_GetObjectItem(root, "cluster");
        const cJSON *nd = cJSON_GetObjectItem(root, "node");
        const cJSON *f2 = cJSON_GetObjectItem(root, "field2d");
        const cJSON *ifs = cJSON_GetObjectItem(root, "ifaces");
        const cJSON *mo = cJSON_GetObjectItem(root, "mode");
        if (!cl || !nd || !f2) { snprintf(err, errlen, "cluster/node/field2d missing"); break; }
        *gen = (uint32_t)num_d(root, "gen", 0);
        *idx = num_i(nd, "idx", -1);
        s_c.steps = (uint32_t)num_d(cl, "steps", 0);
        s_c.alpha = num_d(cl, "alpha", 1.0);
        s_c.dt = num_d(cl, "dt", 1e-4);
        s_c.step_period_ms = num_i(cl, "step_period_ms", 0);
        s_c.nx = (uint32_t)num_i(f2, "nx", 0);
        s_c.ny = (uint32_t)num_i(f2, "ny", 0);
        s_c.ix0 = (uint32_t)num_i(f2, "ix0", 0);
        s_c.iy0 = (uint32_t)num_i(f2, "iy0", 0);
        s_c.h = num_d(f2, "h", 0.0);
        if (s_c.nx == 0 || s_c.ny == 0 || s_c.steps == 0 || !(s_c.h > 0.0)) {
            snprintf(err, errlen, "bad nx/ny/h/steps"); break;
        }
        for (int f = 0; f < 4; f++) s_c.face_kind[f] = F2_BC_ZERO_GRAD;
        const cJSON *faces = cJSON_GetObjectItem(f2, "faces");
        const cJSON *e;
        bool bad = false;
        cJSON_ArrayForEach(e, faces) {
            const cJSON *fc = cJSON_GetObjectItem(e, "face");
            const cJSON *ky = cJSON_GetObjectItem(e, "key");
            if (!cJSON_IsString(fc) || !cJSON_IsString(ky)) { bad = true; break; }
            const char *fs = fc->valuestring;
            int f = !strcmp(fs, "w") ? 0 : !strcmp(fs, "e") ? 1 : !strcmp(fs, "s") ? 2
                  : !strcmp(fs, "n") ? 3 : -1;
            if (f < 0) { bad = true; break; }
            if (!strcmp(ky->valuestring, "interface")) {
                s_c.face_kind[f] = F2_BC_INTERFACE;
                s_c.face_param[f] = (uint64_t)num_i(e, "iface", -1);
            } else if (!strcmp(ky->valuestring, "dirichlet")) {
                double v = num_d(e, "value", 0.0);
                s_c.face_kind[f] = F2_BC_DIRICHLET;
                memcpy(&s_c.face_param[f], &v, 8); /* f64 のビット列 */
            } else if (!strcmp(ky->valuestring, "zero_grad")) {
                s_c.face_kind[f] = F2_BC_ZERO_GRAD;
            } else { bad = true; break; }
        }
        if (bad) { snprintf(err, errlen, "bad face entry"); break; }
        const cJSON *ac = cJSON_GetObjectItem(f2, "cfg");
        cJSON_ArrayForEach(e, ac) {
            if (s_c.n_cfg >= V2_MAX_ACTOR_CFG || !cJSON_IsArray(e) || cJSON_GetArraySize(e) != 2) {
                bad = true; break;
            }
            s_c.cfg_key[s_c.n_cfg] = (uint32_t)cJSON_GetArrayItem(e, 0)->valuedouble;
            s_c.cfg_val[s_c.n_cfg] = cJSON_GetArrayItem(e, 1)->valuedouble;
            s_c.n_cfg++;
        }
        if (bad || (s_c.n_cfg > 0 && !f_cfg)) { snprintf(err, errlen, "bad actor cfg"); break; }
        cJSON_ArrayForEach(e, ifs) {
            if (s_c.n_ifaces >= V2_MAX_IFACES) { bad = true; break; }
            const cJSON *id = cJSON_GetObjectItem(e, "id");
            const cJSON *pr = cJSON_GetObjectItem(e, "peer");
            char ip[24];
            int port = 0;
            if (!cJSON_IsNumber(id) || !cJSON_IsString(pr)
                || sscanf(pr->valuestring, "%23[^:]:%d", ip, &port) != 2) { bad = true; break; }
            struct sockaddr_in *p = &s_c.ifaces[s_c.n_ifaces].peer;
            memset(p, 0, sizeof(*p));
            p->sin_family = AF_INET;
            p->sin_port = htons(port);
            p->sin_addr.s_addr = inet_addr(ip);
            s_c.ifaces[s_c.n_ifaces].id = (uint32_t)id->valueint;
            s_c.n_ifaces++;
        }
        if (bad) { snprintf(err, errlen, "bad iface entry"); break; }
        /* 界面の面と結線の対応: 界面の面ごとに同じIDの結線がちょうど1つ */
        int n_if_faces = 0;
        for (int f = 0; f < 4; f++) {
            if (s_c.face_kind[f] != F2_BC_INTERFACE) continue;
            n_if_faces++;
            int hit = 0;
            for (int i = 0; i < s_c.n_ifaces; i++) hit += s_c.ifaces[i].id == (uint32_t)s_c.face_param[f];
            if (hit != 1) bad = true;
        }
        if (bad || n_if_faces != s_c.n_ifaces) { snprintf(err, errlen, "iface/face mismatch"); break; }
        s_c.field_every = mo ? num_i(mo, "field_every", 50) : 50;
        s_c.rt_every = mo ? num_i(mo, "rt_every", 100) : 100;
        s_c.shape_every = mo ? num_i(mo, "shape_every", f_get_aux ? V2_SHAPE_DEFAULT_EVERY : 0)
                             : (f_get_aux ? V2_SHAPE_DEFAULT_EVERY : 0);
        if (s_c.shape_every > 0 && !f_get_aux) { snprintf(err, errlen, "shape_every needs dyne_get_aux"); break; }
        s_c.gnx = (uint32_t)num_i(f2, "gnx", 0);
        s_c.gny = (uint32_t)num_i(f2, "gny", 0);
        if (s_c.nx > 65535 || s_c.ny > 65535 || s_c.ix0 > 65535 || s_c.iy0 > 65535 || s_c.gnx > 65535
            || s_c.gny > 65535) { snprintf(err, errlen, "grid too large for shape msg"); break; }
        const cJSON *fd = cJSON_GetObjectItem(root, "feed");
        if (fd) {
            s_c.feed_range = cJSON_IsTrue(cJSON_GetObjectItem(fd, "range"));
            s_c.feed_imu = cJSON_IsTrue(cJSON_GetObjectItem(fd, "imu"));
        }
        if (s_c.feed_range && !f_cfg) { snprintf(err, errlen, "feed.range needs dyne_cfg"); break; }
        if (s_c.feed_imu && !f_imu_push) { snprintf(err, errlen, "feed.imu needs dyne_imu_push"); break; }
        /* 2026-11-15: 再生の記録を実行ごとに切り替える (GitHub issue #2) */
        const cJSON *rp = cJSON_GetObjectItem(root, "replay");
        s_c.replay_log = rp && cJSON_IsTrue(cJSON_GetObjectItem(rp, "log"));
        /* 2026-11-13: IMU の融合の実行 (開始の時刻・実時間に合わせた進み方・わざとずらす設定) */
        s_c.imu_time_mode = -1;
        s_c.imu_start_delay_ms = 3000;
        s_c.imu_grid_ms = 10000;
        if (s_c.feed_imu) {
            int per = (int)lround(s_c.dt * 1000.0);
            if (fabs(s_c.dt * 1000.0 - per) > 1e-6 || per <= 0) { snprintf(err, errlen, "imu run: dt must be whole ms"); break; }
            if (s_c.step_period_ms == 0) s_c.step_period_ms = per;
            else if (s_c.step_period_ms != per) { snprintf(err, errlen, "imu run: step_period_ms must equal dt"); break; }
            for (int j = 0; j < s_c.n_cfg; j++)
                if (s_c.cfg_key[j] == IMU_CFG_T0_US) { s_c.imu_t0 = (int64_t)s_c.cfg_val[j]; s_c.imu_t0_given = true; }
            const cJSON *im = cJSON_GetObjectItem(root, "imu");
            if (im) {
                const cJSON *t = cJSON_GetObjectItem(im, "t0_us");
                if (cJSON_IsNumber(t)) { s_c.imu_t0 = (int64_t)t->valuedouble; s_c.imu_t0_given = true; }
                s_c.imu_start_delay_ms = num_i(im, "start_delay_ms", 3000);
                s_c.imu_grid_ms = num_i(im, "grid_ms", 10000);
                const cJSON *tm = cJSON_GetObjectItem(im, "time_mode");
                if (cJSON_IsString(tm)) {
                    if (!strcmp(tm->valuestring, "stamp")) s_c.imu_time_mode = 0;
                    else if (!strcmp(tm->valuestring, "arrival")) s_c.imu_time_mode = 1;
                    else { snprintf(err, errlen, "imu.time_mode: stamp|arrival"); break; }
                }
                const cJSON *o1 = cJSON_GetObjectItem(im, "offset_us"), *o2 = cJSON_GetObjectItem(im, "offset_per_idx_us");
                if (cJSON_IsNumber(o1) || cJSON_IsNumber(o2)) {
                    s_c.imu_off_given = true;
                    s_c.imu_off_us = (cJSON_IsNumber(o1) ? o1->valuedouble : 0.0)
                                   + (cJSON_IsNumber(o2) ? o2->valuedouble : 0.0) * (double)*idx;
                }
            }
        }
        ok = true;
    } while (0);
    cJSON_Delete(root);
    return ok;
}

void dyne_v2_on_cfg(const char *payload, int len)
{
    char err[96] = "", sub[32], json[160];
    uint32_t gen = 0;
    int idx = -1;
    snprintf(sub, sizeof(sub), "cfg/%d/ack", dyne_node_id());
    if (parse_cfg_v2(payload, len, &gen, &idx, err, sizeof(err))) {
        /* 共通部 (start の受理判定・発行のトピック名) は g_cfg に置く */
        memset(&g_cfg, 0, sizeof(g_cfg));
        g_cfg.valid = true;
        g_cfg.gen = gen;
        g_cfg.idx = idx;
        g_cfg.role = 0;
        g_cfg.steps = s_c.steps;
        snprintf(json, sizeof(json), "{\"ok\":true,\"idx\":%d,\"gen\":%u,\"crc32\":\"0x%08X\",\"wire\":2}",
                 idx, (unsigned)gen, (unsigned)dyne_crc32((const uint8_t *)payload, len));
        dyne_mqtt_pub(sub, json, 1, false, false);
        char st[48];
        snprintf(st, sizeof(st), "\"gen\":%u", (unsigned)gen);
        dyne_mqtt_pub_status(DYNE_ST_CONFIGURED, st);
        ESP_LOGI(TAG, "cfg accepted: idx=%d %ux%u at (%u,%u) ifaces=%d", idx,
                 (unsigned)s_c.nx, (unsigned)s_c.ny, (unsigned)s_c.ix0, (unsigned)s_c.iy0, s_c.n_ifaces);
    } else {
        g_cfg.valid = false;
        snprintf(json, sizeof(json), "{\"ok\":false,\"idx\":%d,\"err\":\"%s\"}", dyne_node_id(), err);
        dyne_mqtt_pub(sub, json, 1, false, false);
        ESP_LOGE(TAG, "cfg rejected: %s", err);
    }
}

/* ==== BSP 協調 (run_bsp と同じ組み立て) ==== */

typedef struct {
    uint32_t id;
    struct sockaddr_in peer;
    struct { uint64_t step; int len; bool valid; uint8_t buf[DYNE_ENV_MAX_DATAGRAM]; } slot[2];
    struct { uint64_t step; size_t len; bool valid; uint8_t buf[DYNE_ENV_MAX_PAYLOAD]; } pend;
    uint8_t unit[DYNE_ENV_MAX_PAYLOAD]; /* 送信する配送単位の組み立て */
    size_t unit_len;
    bool got;
    int64_t last_send;
} v2link_t;

static uint64_t st_sent, st_resend, st_replay, st_bad_env, st_rejected, st_missing;
/* 2026-11-13: センサの受け渡しの計数 */
static uint64_t st_shape_sent, st_shape_skip; /* 2026-11-13 */
static uint64_t st_fused_sent; /* 2026-11-13 */
static int64_t s_imu_t0_run, s_imu_late_start_ms;
static uint64_t st_rf_fed, st_rf_skip, st_rf_rej, st_imu_pushed, st_imu_late, st_imu_early, st_imu_lost,
    st_imu_wait_to;

static uint32_t crc_update(uint32_t c, const uint8_t *d, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++) {
            uint32_t m = -(c & 1u);
            c = (c >> 1) ^ (0xEDB88320u & m);
        }
    }
    return c;
}

/* 状態 (dyne_get の全点、f64 のバイト列) の CRC32 と最小・最大。1次元の経路の
 * 「温度配列バイト列のCRC32」と同じ考え方 (シミュレータの同じ区画と比べる) */
static uint32_t state_crc(double *vmin, double *vmax)
{
    uint32_t n = call_u32(f_len), c = 0xFFFFFFFFu;
    *vmin = INFINITY; *vmax = -INFINITY;
    for (uint32_t k = 0; k < n; k++) {
        double u = get_k(k);
        uint8_t b[8];
        memcpy(b, &u, 8); /* ESP32 系はリトルエンディアン = シミュレータと同じ並び */
        c = crc_update(c, b, 8);
        if (u < *vmin) *vmin = u;
        if (u > *vmax) *vmax = u;
    }
    return ~c;
}

static void publish_summary(uint64_t step)
{
    double lo, hi;
    uint32_t crc = state_crc(&lo, &hi);
    char sub[32], json[200];
    snprintf(sub, sizeof(sub), "node/%d/field2d", g_cfg.idx);
    snprintf(json, sizeof(json),
             "{\"step\":%llu,\"nx\":%u,\"ny\":%u,\"ix0\":%u,\"iy0\":%u,\"crc32\":\"0x%08X\","
             "\"min\":%.17g,\"max\":%.17g}",
             (unsigned long long)step, (unsigned)s_c.nx, (unsigned)s_c.ny, (unsigned)s_c.ix0,
             (unsigned)s_c.iy0, (unsigned)crc, lo, hi);
    dyne_mqtt_pub(sub, json, 0, false, true);
}

static uint8_t *actor_buf(uint32_t app, uint32_t len)
{
    if (!wasm_runtime_validate_app_addr(s_inst, app, len)) return NULL;
    return (uint8_t *)wasm_runtime_addr_app_to_native(s_inst, app);
}

/* 受信した配送単位 (枠つきメッセージ列) を受信メールボックスへ渡す */
static int deliver(uint32_t app, uint32_t cap, const uint8_t *payload, size_t plen)
{
    if (plen + 4 > cap) return -1;
    uint8_t *p = actor_buf(app, (uint32_t)plen + 4);
    if (!p) return -1;
    memcpy(p, payload, plen);
    memset(p + plen, 0, 4); /* 終端印 */
    wasm_val_t a[2], r[1];
    a[0].kind = WASM_I32; a[0].of.i32 = (int32_t)app;
    a[1].kind = WASM_I32; a[1].of.i32 = (int32_t)(plen + 4);
    wasm_runtime_call_wasm_a(s_env, f_inbox, 1, r, 2, a);
    return r[0].of.i32;
}

static void send_slot(int sock, v2link_t *l, int si)
{
    sendto(sock, l->slot[si].buf, l->slot[si].len, 0, (struct sockaddr *)&l->peer, sizeof(l->peer));
}

/* 送信メールボックスを取り出し、界面ごとの配送単位にまとめて外箱に包んで送る。
 * 戻り値: NULL = 成功 / 誤りの説明 */
static const char *send_step(int sock, v2link_t *links, int nl, uint32_t app, uint32_t cap,
                             uint64_t step, uint32_t seq)
{
    wasm_val_t a[2], r[1];
    a[0].kind = WASM_I32; a[0].of.i32 = (int32_t)app;
    a[1].kind = WASM_I32; a[1].of.i32 = (int32_t)cap;
    wasm_runtime_call_wasm_a(s_env, f_outbox, 1, r, 2, a);
    uint32_t n = (uint32_t)r[0].of.i32;
    if (n > cap) return "outbox len";
    const uint8_t *ob = actor_buf(app, n ? n : 1);
    if (!ob) return "outbox addr";
    for (int i = 0; i < nl; i++) links[i].unit_len = 0;
    /* 2026-11-11: ヘッダの時刻印の欄 (t_us、オフセット16) が 0 (アクターが付けていない) の
     * メッセージに、送信メールボックスを読み出した時点 = この境界の値ができた時点の
     * 同期した時刻を付ける。未同期なら 0 のまま。受け手のアクターは t_us を計算に使わないので
     * 数値の結果は変わらない */
    int64_t stamp = dyne_time_now_us();
    size_t pos = 0;
    while (pos + 4 <= n) {
        uint32_t mlen = (uint32_t)ob[pos] | ((uint32_t)ob[pos + 1] << 8)
                      | ((uint32_t)ob[pos + 2] << 16) | ((uint32_t)ob[pos + 3] << 24);
        if (mlen == 0) break;
        if (mlen > n - pos - 4) return "outbox frame";
        uint8_t kind; uint32_t src; uint64_t mstep;
        if (dyne_v2_peek(ob + pos + 4, mlen, &kind, &src, &mstep) != 0) return "outbox msg";
        if (!dyne_v2_routed_by_iface(kind) || mstep != step) return "outbox kind/step";
        v2link_t *l = NULL;
        for (int i = 0; i < nl; i++) if (links[i].id == src) l = &links[i];
        if (!l) return "outbox iface";
        if (l->unit_len + 4 + mlen > DYNE_ENV_MAX_PAYLOAD) return "unit oversize";
        memcpy(l->unit + l->unit_len, ob + pos, 4 + mlen);
        if (stamp > 0 && mlen >= DYNE_V2_HDR_LEN) {
            uint8_t *t = l->unit + l->unit_len + 4 + 16;
            bool zero = true;
            for (int k = 0; k < 8; k++) zero &= t[k] == 0;
            if (zero)
                for (int k = 0; k < 8; k++) t[k] = (uint8_t)((uint64_t)stamp >> (8 * k));
        }
        l->unit_len += 4 + mlen;
        pos += 4 + mlen;
    }
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < nl; i++) {
        v2link_t *l = &links[i];
        if (l->unit_len == 0) return "no message for iface";
        int si = (int)(step & 1);
        dyne_env_hdr_t h = { .sender = (uint32_t)dyne_node_id(), .seq = seq, .iface = l->id, .step = step };
        int dn = dyne_env_encode(l->slot[si].buf, sizeof(l->slot[si].buf), &h, l->unit, l->unit_len);
        if (dn < 0) return "envelope encode";
        l->slot[si].len = dn;
        l->slot[si].step = step;
        l->slot[si].valid = true;
        l->got = false;
        send_slot(sock, l, si);
        st_sent++;
        l->last_send = now;
    }
    return NULL;
}

static double get_aux_k(uint32_t q, uint32_t k)
{
    wasm_val_t a[2] = { { .kind = WASM_I32, .of.i32 = (int32_t)q }, { .kind = WASM_I32, .of.i32 = (int32_t)k } };
    wasm_val_t r[1];
    wasm_runtime_call_wasm_a(s_env, f_get_aux, 1, r, 2, a);
    return r[0].of.f64;
}

/* 注入された形のアクターの設定から値を探す (無ければ def) */
static double actor_cfg_val(uint32_t key, double def)
{
    double v = def;
    for (int j = 0; j < s_c.n_cfg; j++)
        if (s_c.cfg_key[j] == key) v = s_c.cfg_val[j]; /* 後の注入が勝つ (アクターと同じ) */
    return v;
}

/* 2026-11-13: 形の区画のメッセージ (docs/2026-11-13-pc-pipeline-rl.md §5.2) を dyne/node/<idx>/shape へ */
static void publish_shape(uint64_t step)
{
    uint32_t n = call_u32(f_len);
    if (n != s_c.nx * s_c.ny) { st_shape_skip++; return; }
    size_t len = dyne_shape_msg_len((uint16_t)s_c.nx, (uint16_t)s_c.ny);
    uint8_t *buf = malloc(len); /* 縮小版で最大 52 + 12·1080 = 13KB。送ったらすぐ放す */
    if (!buf) { st_shape_skip++; return; }
    for (uint32_t k = 0; k < n; k++) {
        dyne_shape_put(buf, k, (float)get_k(k));
        dyne_shape_put(buf, n + k, (float)get_aux_k(SDF_AUX_D, k));
        dyne_shape_put(buf, 2 * n + k, (float)get_aux_k(SDF_AUX_W, k));
    }
    int64_t now = dyne_time_now_us();
    dyne_shape_hdr_t h = {
        .node = (uint16_t)g_cfg.idx, .step = (uint32_t)step, .t_us = now > 0 ? (uint64_t)now : 0,
        .ix0 = (uint16_t)s_c.ix0, .iy0 = (uint16_t)s_c.iy0, .nx = (uint16_t)s_c.nx, .ny = (uint16_t)s_c.ny,
        .gnx = (uint16_t)s_c.gnx, .gny = (uint16_t)s_c.gny, .h = (float)s_c.h,
        .x0 = (float)actor_cfg_val(SDF_CFG_ORIGIN_X, 0.0), .y0 = (float)actor_cfg_val(SDF_CFG_ORIGIN_Y, 0.0),
    };
    dyne_shape_finish(buf, &h, dyne_env_crc32);
    char sub[32];
    snprintf(sub, sizeof(sub), "node/%d/shape", g_cfg.idx);
    if (dyne_mqtt_pub_bin(sub, buf, (int)len, 0, V2_SHAPE_MAX_OUTBOX) == 0) st_shape_sent++;
    else st_shape_skip++;
    free(buf);
}

/* 2026-11-13: 融合の出力 (区間の番号 dyne_get(0) が変わったら) を dyne/node/<id>/imu_fused へ。
 * id は生の標本 (dyne/node/<id>/imu、dyne_imu.c) と同じ dyne_node_id() (PC の imu_log.py が両方をノードごとに並べる) */
static void publish_fused(uint64_t step, double *last_epoch)
{
    double e = get_k(0);
    if (e == *last_epoch || e < 0) return;
    *last_epoch = e;
    uint32_t n = call_u32(f_len);
    if (n > IMU_FUSED_MAX) n = IMU_FUSED_MAX;
    double v[IMU_FUSED_MAX];
    for (uint32_t k = 0; k < n; k++) v[k] = get_k(k);
    char js[IMU_FUSED_MAX * 26 + 48], sub[32];
    if (dyne_imu_fused_json(js, sizeof(js), step, v, (int)n) < 0) return;
    snprintf(sub, sizeof(sub), "node/%d/imu_fused", dyne_node_id());
    dyne_mqtt_pub(sub, js, 0, false, true);
    st_fused_sent++;
}

/* 2026-11-15: 再生の記録 (実行の設定の "replay":{"log":true} のときだけ。既定 off。GitHub issue #2、
 * docs/2026-11-15-hw-replay.md)。PC が記録して dyne-sim2d で同じ入力を再生し、ノードの報告した CRC とビット一致を
 * 確かめるため。menuconfig ではなく実行ごとに切り替える (同じファームで記録する・しないを選べる。コードは小さい)。
 *  - dyne/node/<id>/actor_setup: 実行の始めに1回。アクターへの dyne_cfg の全呼び出し (順に [キー, 値])・面・初期化の値
 *    {"idx":i,"id":id,"cfg":[[k,v],...],"faces":[[kind,param],...],"init":{"nx":..,"ny":..,"ix0":..,"iy0":..,
 *     "h":..,"alpha":..,"dt":..},"trunc":false}  (値は %.17g = f64 を正確に戻せる桁。faces は面 W, E, S, N の順)
 *  - dyne/node/<id>/imu_fed: dyne_imu_push に渡した標本 (戻り値によらず全て)。ステップごと (多ければ分ける)
 *    {"step":s,"i0":そのステップの中での最初の標本の番号,"s":[[t_us,ax,ay,az,gx,gy,gz],...]}
 *  - dyne/node/<id>/range_fed: 受信メールボックスへ渡した測距フレーム (dyne_range.c の dyne_range_publish_fed)
 * どれも QoS 1 で送る (MQTT の取りこぼしで偽の FAIL にしないため)。他の発行 (field2d・rt・shape・imu_fused) は QoS 0 のまま。
 * MQTT のアウトボックスが V2_RL_MAX_OUTBOX を超えていたら送らずに数え (done の replay_skip)、PC は記録を
 * 不完全として扱う (scripts/hw_replay.py)。実行は止めない */
#define RL_SETUP_SZ 4096
#define RL_IMU_SZ   1536
#define V2_RL_MAX_OUTBOX 32768
static char *s_rl_setup, *s_rl_imu;
static int s_rl_setup_n, s_rl_ncfg, s_rl_imu_n;
static bool s_rl_trunc;
static uint32_t s_rl_imu_cnt, s_rl_imu_i0;
static uint64_t st_rl_skip; /* 送らなかった再生の記録の数 */

static void rl_pub(const char *kind, const char *json)
{
    char sub[40];
    snprintf(sub, sizeof(sub), "node/%d/%s", dyne_node_id(), kind);
    if (dyne_mqtt_pub_bin(sub, json, (int)strlen(json), 1, V2_RL_MAX_OUTBOX) != 0) st_rl_skip++;
}

static void rl_put(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void rl_put(const char *fmt, ...)
{
    if (!s_rl_setup || s_rl_trunc) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_rl_setup + s_rl_setup_n, RL_SETUP_SZ - s_rl_setup_n, fmt, ap);
    va_end(ap);
    if (n < 0 || s_rl_setup_n + n >= RL_SETUP_SZ - 32) s_rl_trunc = true; /* 末尾の閉じ括弧の余地を残す */
    else s_rl_setup_n += n;
}

static void rl_setup_begin(void)
{
    free(s_rl_setup);
    s_rl_setup = NULL;
    st_rl_skip = 0;
    if (!s_c.replay_log) return;
    s_rl_setup = malloc(RL_SETUP_SZ);
    if (!s_rl_setup) { st_rl_skip++; return; }
    s_rl_setup_n = s_rl_ncfg = 0;
    s_rl_trunc = false;
    rl_put("{\"idx\":%d,\"id\":%d,\"cfg\":[", g_cfg.idx, dyne_node_id());
}

static void rl_cfg(uint32_t key, double val)
{
    rl_put("%s[%u,%.17g]", s_rl_ncfg++ ? "," : "", (unsigned)key, val);
}

static void rl_setup_end(bool publish)
{
    if (!s_rl_setup) return;
    if (publish) {
        rl_put("],\"faces\":[");
        for (int f = 0; f < 4; f++)
            rl_put("%s[%d,%llu]", f ? "," : "", s_c.face_kind[f], (unsigned long long)s_c.face_param[f]);
        rl_put("],\"init\":{\"nx\":%u,\"ny\":%u,\"ix0\":%u,\"iy0\":%u,\"h\":%.17g,\"alpha\":%.17g,\"dt\":%.17g}",
               (unsigned)s_c.nx, (unsigned)s_c.ny, (unsigned)s_c.ix0, (unsigned)s_c.iy0, s_c.h, s_c.alpha, s_c.dt);
        if (s_rl_trunc) /* 入りきらなかった: 再生に使えないことだけを伝える */
            s_rl_setup_n = snprintf(s_rl_setup, RL_SETUP_SZ, "{\"idx\":%d,\"id\":%d,\"trunc\":true}", g_cfg.idx,
                                    dyne_node_id());
        else
            snprintf(s_rl_setup + s_rl_setup_n, RL_SETUP_SZ - s_rl_setup_n, ",\"trunc\":false}");
        rl_pub("actor_setup", s_rl_setup);
    }
    free(s_rl_setup);
    s_rl_setup = NULL;
}

static void rl_imu_flush(void)
{
    if (!s_rl_imu || s_rl_imu_n == 0) return;
    snprintf(s_rl_imu + s_rl_imu_n, RL_IMU_SZ - s_rl_imu_n, "]}");
    rl_pub("imu_fed", s_rl_imu);
    s_rl_imu_n = 0;
    s_rl_imu_i0 = s_rl_imu_cnt;
}

static void rl_imu_add(uint64_t step, const dyne_imu_sample_t *s)
{
    if (!s_rl_imu) return;
    if (s_rl_imu_n == 0)
        s_rl_imu_n = snprintf(s_rl_imu, RL_IMU_SZ, "{\"step\":%llu,\"i0\":%u,\"s\":[", (unsigned long long)step,
                              (unsigned)s_rl_imu_i0);
    else
        s_rl_imu[s_rl_imu_n++] = ',';
    s_rl_imu_n += snprintf(s_rl_imu + s_rl_imu_n, RL_IMU_SZ - s_rl_imu_n,
                           "[%lld,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]", (long long)s->t_sync_us, s->a[0], s->a[1],
                           s->a[2], s->g[0], s->g[1], s->g[2]);
    s_rl_imu_cnt++;
    if (s_rl_imu_n > RL_IMU_SZ - 200) rl_imu_flush(); /* 1標本は最大約 180 文字 */
}

static void cfg_call(uint32_t key, double val)
{
    wasm_val_t a[2];
    a[0].kind = WASM_I32; a[0].of.i32 = (int32_t)key;
    a[1].kind = WASM_F64; a[1].of.f64 = val;
    wasm_runtime_call_wasm_a(s_env, f_cfg, 0, NULL, 2, a);
    rl_cfg(key, val);
}

/* 2026-11-13: 測距の最新のフレームを RANGE_FRAME として受信メールボックスへ */
static void feed_range(uint32_t app, uint32_t cap, uint64_t step, uint32_t *last_seq)
{
    dyne_range_frame_t f;
    uint32_t skipped = 0;
    if (!dyne_range_take(&f, last_seq, &skipped)) return;
    st_rf_skip += skipped;
    uint8_t buf[4 + 24 + 8 + DYNE_RANGE_ZONE_LEN * DYNE_RANGE_MAX_ZONES];
    uint64_t t_us = f.t_sync_us > 0 ? (uint64_t)f.t_sync_us : 0;
    int n = dyne_range_frame_build(buf, sizeof(buf), f.sensor_id, step, t_us, f.d_mm, f.st, f.count, f.cols);
    if (n < 0) { st_rf_rej++; return; }
    if (deliver(app, cap, buf, (size_t)n) != F2_INBOX_OK) st_rf_rej++;
    else st_rf_fed++;
    /* 2026-11-15: 再生の記録 (replay.log のときだけ。QoS 1) */
    if (s_c.replay_log && dyne_range_publish_fed(&f, step, V2_RL_MAX_OUTBOX) != 0) st_rl_skip++;
}

/* 2026-11-13: IMU の標本を dyne_imu_push で入れる (アクターが示した時刻までは待つ) */
static void feed_imu(uint64_t *rd, uint64_t step)
{
    if (f_imu_need) {
        wasm_val_t r[1];
        wasm_runtime_call_wasm_a(s_env, f_imu_need, 1, r, 0, NULL);
        int64_t need = r[0].of.i64;
        if (need > 0) {
            int64_t until = esp_timer_get_time() + (int64_t)V2_IMU_WAIT_MS * 1000;
            while (dyne_imu_newest_t() < need) {
                if (esp_timer_get_time() > until) { st_imu_wait_to++; break; }
                vTaskDelay(pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1);
            }
        }
    }
    dyne_imu_sample_t s[V2_IMU_BATCH]; /* 約1.8KB。BSP は 32KB のスタックの node_task で動く */
    for (;;) {
        uint64_t lost = 0;
        int n = dyne_imu_take(s, V2_IMU_BATCH, rd, &lost);
        st_imu_lost += lost;
        for (int i = 0; i < n; i++) {
            wasm_val_t a[7], r[1];
            a[0].kind = WASM_I64; a[0].of.i64 = s[i].t_sync_us;
            for (int k = 0; k < 3; k++) {
                a[1 + k].kind = WASM_F64; a[1 + k].of.f64 = s[i].a[k];
                a[4 + k].kind = WASM_F64; a[4 + k].of.f64 = s[i].g[k];
            }
            wasm_runtime_call_wasm_a(s_env, f_imu_push, 1, r, 7, a);
            rl_imu_add(step, &s[i]); /* 2026-11-15: 再生の記録 (replay.log のときだけ領域がある) */
            int rc = r[0].of.i32;
            if (rc == 0) st_imu_pushed++;
            else if (rc == 1) st_imu_late++;
            else if (rc == 2) st_imu_early++;
        }
        if (n < V2_IMU_BATCH) break;
    }
    rl_imu_flush(); /* そのステップの分を送る (次のステップの番号と混ぜない) */
    s_rl_imu_cnt = s_rl_imu_i0 = 0;
}

void dyne_v2_run_bsp(void)
{
    char sub[32], json[256];
    const char *fail = NULL;
    int nl = s_c.n_ifaces;
    v2link_t *links = calloc(nl > 0 ? nl : 1, sizeof(v2link_t)); /* 実行中だけ確保 */
    uint8_t *rb = malloc(DYNE_ENV_MAX_DATAGRAM + 1);
    if (!links || !rb) {
        free(links); free(rb);
        dyne_mqtt_pub_status(DYNE_ST_ERR, "\"err\":\"v2 alloc\"");
        return;
    }
    for (int i = 0; i < nl; i++) {
        links[i].id = s_c.ifaces[i].id;
        links[i].peer = s_c.ifaces[i].peer;
    }

    /* アクター初期化 (dyne_cfg → 面の設定 → dyne_init2d。dyne-sim2d と同じ順序) */
    rl_setup_begin(); /* 2026-11-15: 再生の記録 (replay.log のときだけ) */
    /* 2026-11-15: 同じ呼び出しを cfg_call に寄せた (再生の記録に載せるため。呼び出しの中身は同じ) */
    for (int j = 0; j < s_c.n_cfg; j++) cfg_call(s_c.cfg_key[j], s_c.cfg_val[j]);
    /* 2026-11-13: IMU の融合の実行: 開始の時刻 t0 と、わざとずらす設定 (PC の設定の後に注入して上書き) */
    s_imu_t0_run = 0;
    s_imu_late_start_ms = 0;
    if (s_c.feed_imu) {
        if (!dyne_time_synced()) {
            dyne_mqtt_pub_status(DYNE_ST_ERR, "\"err\":\"imu run needs time sync (not synced yet)\"");
            rl_setup_end(false);
            free(links); free(rb);
            return;
        }
        int64_t t0 = s_c.imu_t0_given ? s_c.imu_t0
                   : dyne_imu_auto_t0(dyne_time_now_us(), (int64_t)s_c.imu_start_delay_ms * 1000,
                                      (int64_t)s_c.imu_grid_ms * 1000);
        s_imu_t0_run = t0;
        cfg_call(IMU_CFG_T0_US, (double)t0);
        if (s_c.imu_time_mode >= 0) cfg_call(IMU_CFG_TIME_MODE, s_c.imu_time_mode);
        if (s_c.imu_off_given) cfg_call(IMU_CFG_TS_OFFSET_US, s_c.imu_off_us);
        char st[160];
        snprintf(st, sizeof(st), "\"imu_t0_us\":%lld,\"imu_time_mode\":%d,\"imu_offset_us\":%.0f,\"step_period_ms\":%d",
                 (long long)t0, s_c.imu_time_mode, s_c.imu_off_given ? s_c.imu_off_us : 0.0, s_c.step_period_ms);
        dyne_mqtt_pub_status(DYNE_ST_RUNNING, st);
        ESP_LOGI(TAG, "imu run: t0=%lld offset=%.0f mode=%d", (long long)t0, s_c.imu_off_us, s_c.imu_time_mode);
    }
    /* 2026-11-13: 測距のセンサの一覧 (dyne-sim2d と同じく、PC の設定の後に注入する) */
    if (s_c.feed_range) {
        int zr = dyne_range_actor_zone_row();
        if (zr >= 0) cfg_call(SDF_CFG_ZONE_ROW, zr);
        uint32_t id[16];
        double sx[16], sy[16], syaw[16];
        int ns = dyne_range_sensors(id, sx, sy, syaw, 16);
        for (int k = 0; k < ns; k++) {
            cfg_call(SDF_CFG_SENSOR_SEL, id[k]);
            cfg_call(SDF_CFG_SENSOR_X, sx[k]);
            cfg_call(SDF_CFG_SENSOR_Y, sy[k]);
            cfg_call(SDF_CFG_SENSOR_YAW, syaw[k]);
            ESP_LOGI(TAG, "sensor id=%u at (%.3f, %.3f) yaw=%.4f rad", (unsigned)id[k], sx[k], sy[k], syaw[k]);
        }
    }
    for (int f = 0; f < 4; f++) {
        wasm_val_t a[3];
        a[0].kind = WASM_I32; a[0].of.i32 = f;
        a[1].kind = WASM_I32; a[1].of.i32 = s_c.face_kind[f];
        a[2].kind = WASM_I64; a[2].of.i64 = (int64_t)s_c.face_param[f];
        wasm_runtime_call_wasm_a(s_env, f_set_face, 0, NULL, 3, a);
    }
    {
        wasm_val_t a[7], r[1];
        a[0].kind = WASM_I32; a[0].of.i32 = (int32_t)s_c.nx;
        a[1].kind = WASM_I32; a[1].of.i32 = (int32_t)s_c.ny;
        a[2].kind = WASM_I32; a[2].of.i32 = (int32_t)s_c.ix0;
        a[3].kind = WASM_I32; a[3].of.i32 = (int32_t)s_c.iy0;
        a[4].kind = WASM_F64; a[4].of.f64 = s_c.h;
        a[5].kind = WASM_F64; a[5].of.f64 = s_c.alpha;
        a[6].kind = WASM_F64; a[6].of.f64 = s_c.dt;
        if (!wasm_runtime_call_wasm_a(s_env, f_init2d, 1, r, 7, a) || r[0].of.i32 != 0) {
            snprintf(json, sizeof(json), "\"err\":\"dyne_init2d failed rc=%d\"", (int)r[0].of.i32);
            rl_setup_end(false);
            dyne_mqtt_pub_status(DYNE_ST_ERR, json);
            free(links); free(rb);
            return;
        }
    }
    rl_setup_end(true); /* 2026-11-15: dyne/node/<id>/actor_setup (replay.log のときだけ) */
    if (s_c.replay_log && s_c.feed_imu && !s_rl_imu) { /* 実行の間だけ */
        s_rl_imu = malloc(RL_IMU_SZ);
        if (!s_rl_imu) st_rl_skip++; /* 記録できない (PC は imu_fed の数の食い違いでも気づく) */
    }
    s_rl_imu_n = 0;
    s_rl_imu_cnt = s_rl_imu_i0 = 0;
    uint32_t app = call_u32(f_msg_buf), cap = call_u32(f_msg_cap);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in me = { 0 };
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    me.sin_port = htons(CONFIG_DYNE_UDP_PORT);
    bind(sock, (struct sockaddr *)&me, sizeof(me));
    struct timeval tv = { .tv_sec = 0, .tv_usec = V2_RECV_SLICE_MS * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    st_sent = st_resend = st_replay = st_bad_env = st_rejected = st_missing = 0;
    st_rf_fed = st_rf_skip = st_rf_rej = st_imu_pushed = st_imu_late = st_imu_early = st_imu_lost = 0;
    st_imu_wait_to = 0;
    st_shape_sent = st_shape_skip = 0;
    st_fused_sent = 0;
    double fused_epoch = -1.0;
    uint32_t rf_seq = 0;
    uint64_t imu_rd = 0;
    if (s_c.feed_range) { dyne_range_frame_t tmp; uint32_t sk; dyne_range_take(&tmp, &rf_seq, &sk); } /* 開始前のフレームは渡さない */
    uint32_t seq = 0;
    bool aborted = false;
    double w_step = 0, w_comp = 0;
    uint32_t w_n = 0;

    ESP_LOGI(TAG, "BSP(v2) start: %u steps, %d iface(s)", (unsigned)s_c.steps, nl);
    int64_t pace0 = esp_timer_get_time();
    if (s_imu_t0_run > 0) {
        /* 2026-11-13: ステップ0 = 同期した時刻 t0。それまで待ち、t0 に当たる esp_timer の時刻を起点にする */
        for (;;) {
            int64_t ns = dyne_time_now_us();
            if (ns >= s_imu_t0_run || g_stop_req) break;
            int64_t ms = (s_imu_t0_run - ns) / 1000;
            vTaskDelay(pdMS_TO_TICKS(ms > 50 ? 50 : (ms > 1 ? ms : 1)));
        }
        if (g_stop_req) aborted = true;
        int64_t ns = dyne_time_now_us();
        pace0 = esp_timer_get_time() - (ns - s_imu_t0_run);
        s_imu_late_start_ms = (ns - s_imu_t0_run) / 1000; /* 正 = t0 より遅れて始めた */
        imu_rd = 0; /* 開始前の標本は捨てる (t0 より前は区間に入らない) */
        { uint64_t lost; dyne_imu_sample_t tmp[1]; while (dyne_imu_take(tmp, 1, &imu_rd, &lost) > 0) { } }
    }
    for (uint64_t step = 0; step < s_c.steps && !aborted && !fail; step++) {
        if (s_c.step_period_ms > 0) {
            int64_t target = pace0 + (int64_t)step * s_c.step_period_ms * 1000;
            int64_t now = esp_timer_get_time();
            if (now < target) vTaskDelay(pdMS_TO_TICKS((uint32_t)((target - now) / 1000) + 1));
        }
        int64_t t0 = esp_timer_get_time();
        seq++;
        /* 1) 全界面へ送信 (控えを残す) */
        if ((fail = send_step(sock, links, nl, app, cap, step, seq)) != NULL) break;
        /* 1b) ノード内のセンサの入力 (ネットワークを通らない。dyne-sim2d と同じ位置) */
        if (s_c.feed_range) feed_range(app, cap, step, &rf_seq);
        /* 2) 取り置き分の即時消費 */
        for (int i = 0; i < nl && !fail; i++) {
            if (links[i].pend.valid && links[i].pend.step == step) {
                if (deliver(app, cap, links[i].pend.buf, links[i].pend.len) != F2_INBOX_OK) {
                    st_rejected++;
                    fail = "inbox rejected";
                }
                links[i].got = true;
                links[i].pend.valid = false;
            }
        }
        /* 3) 全界面が揃うまで受信・再送 */
        while (!fail) {
            bool all = true;
            for (int i = 0; i < nl; i++) all &= links[i].got;
            if (all) break;
            if (g_stop_req) { aborted = true; break; }
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            /* 上限+1バイトで受け、上限を超えたデータグラムを検出する */
            int rn = recvfrom(sock, rb, DYNE_ENV_MAX_DATAGRAM + 1, 0, (struct sockaddr *)&src, &sl);
            int64_t now = esp_timer_get_time();
            if (rn > 0) {
                dyne_env_hdr_t h;
                const uint8_t *pl;
                size_t pn;
                if (dyne_env_decode(rb, (size_t)rn, &h, &pl, &pn) != 0) { st_bad_env++; continue; }
                v2link_t *l = NULL;
                for (int i = 0; i < nl; i++) if (links[i].id == h.iface) l = &links[i];
                if (!l) continue;
                if (h.step == step && !l->got) {
                    if (deliver(app, cap, pl, pn) != F2_INBOX_OK) {
                        st_rejected++;
                        fail = "inbox rejected";
                    }
                    l->got = true;
                } else if (h.step > step) {
                    memcpy(l->pend.buf, pl, pn);
                    l->pend.len = pn;
                    l->pend.step = h.step;
                    l->pend.valid = true;
                } else if (h.step < step) {
                    /* 過去分 = 相手はまだ受け取れていない。控えを再送で応答 */
                    int ri = (int)(h.step & 1);
                    if (l->slot[ri].valid && l->slot[ri].step == h.step) {
                        send_slot(sock, l, ri);
                        st_replay++;
                    }
                }
            } else {
                for (int i = 0; i < nl; i++) {
                    if (!links[i].got && now - links[i].last_send >= V2_TIMEOUT_US) {
                        send_slot(sock, &links[i], (int)(step & 1));
                        st_resend++;
                        links[i].last_send = now + V2_RESEND_MIN_US - V2_TIMEOUT_US;
                    }
                }
            }
        }
        if (aborted || fail) break;
        int64_t t1 = esp_timer_get_time();
        /* 3b) IMU の標本 (計算の直前) */
        if (s_c.feed_imu) feed_imu(&imu_rd, step);
        /* 4) 計算 */
        wasm_val_t r[1];
        wasm_runtime_call_wasm_a(s_env, f_step, 1, r, 0, NULL);
        if (r[0].of.i32 != F2_STEP_OK) st_missing++;
        if (f_imu_push) publish_fused(step, &fused_epoch); /* 2026-11-13: 区間の境目ごと */
        int64_t t2 = esp_timer_get_time();

        w_step += (t2 - t0) / 1000.0;
        w_comp += (t2 - t1) / 1000.0;
        w_n++;
        if (s_c.field_every > 0 && (step + 1) % (uint64_t)s_c.field_every == 0) publish_summary(step);
        if (s_c.shape_every > 0 && (step + 1) % (uint64_t)s_c.shape_every == 0) publish_shape(step);
        if ((s_c.rt_every > 0 && (step + 1) % (uint64_t)s_c.rt_every == 0) || g_report_req) {
            snprintf(sub, sizeof(sub), "node/%d/rt", g_cfg.idx);
            snprintf(json, sizeof(json),
                     "{\"step\":%llu,\"window\":%u,\"step_ms\":{\"mean\":%.3f},"
                     "\"compute_ms\":{\"mean\":%.3f},\"wire\":2}",
                     (unsigned long long)step, (unsigned)w_n, w_step / w_n, w_comp / w_n);
            dyne_mqtt_pub(sub, json, 0, false, true);
            w_step = w_comp = 0;
            w_n = 0;
            snprintf(json, sizeof(json), "\"step\":%llu", (unsigned long long)(step + 1));
            dyne_mqtt_pub_status(DYNE_ST_RUNNING, json);
            g_report_req = false;
        }
    }

    if (!aborted && !fail) {
        /* 終了時ドレイン: 相手の再要求に応答してから締める */
        int64_t drain_end = esp_timer_get_time() + (int64_t)V2_DRAIN_MS * 1000;
        while (esp_timer_get_time() < drain_end) {
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            int rn = recvfrom(sock, rb, DYNE_ENV_MAX_DATAGRAM + 1, 0, (struct sockaddr *)&src, &sl);
            if (rn <= 0) continue;
            dyne_env_hdr_t h;
            const uint8_t *pl;
            size_t pn;
            if (dyne_env_decode(rb, (size_t)rn, &h, &pl, &pn) != 0) continue;
            for (int i = 0; i < nl; i++) {
                int ri = (int)(h.step & 1);
                if (links[i].id == h.iface && links[i].slot[ri].valid && links[i].slot[ri].step == h.step) {
                    send_slot(sock, &links[i], ri);
                    st_replay++;
                }
            }
        }
    }
    close(sock);
    free(links);
    free(rb);
    free(s_rl_imu);
    s_rl_imu = NULL;

    if (fail) {
        snprintf(json, sizeof(json), "\"err\":\"v2: %s\"", fail);
        dyne_mqtt_pub_status(DYNE_ST_ERR, json);
        ESP_LOGE(TAG, "BSP(v2) failed: %s", fail);
        return;
    }
    if (aborted) {
        g_stop_req = false;
        dyne_mqtt_pub_status(DYNE_ST_CONFIGURED, "\"note\":\"stopped\"");
        ESP_LOGW(TAG, "BSP(v2) stopped by ctl");
        return;
    }
    double lo, hi;
    uint32_t crc = state_crc(&lo, &hi);
    publish_summary(s_c.steps - 1);
    if (s_c.shape_every > 0) publish_shape(s_c.steps - 1); /* 最後の場は必ず送る */
    char extra[1024];
    int en = snprintf(extra, sizeof(extra),
             "\"wire\":2,\"crc32\":\"0x%08X\",\"sent\":%llu,\"resend\":%llu,\"replay\":%llu,"
             "\"bad_env\":%llu,\"rejected\":%llu,\"missing\":%llu,\"dropped\":%llu",
             (unsigned)crc, (unsigned long long)st_sent, (unsigned long long)st_resend,
             (unsigned long long)st_replay, (unsigned long long)st_bad_env,
             (unsigned long long)(st_rejected + call_u64(f_rejected)),
             (unsigned long long)(st_missing + call_u64(f_missing)), (unsigned long long)call_u64(f_dropped));
    if (s_c.feed_imu)
        en += snprintf(extra + en, sizeof(extra) - en, ",\"imu_fused_sent\":%llu,\"imu_t0_us\":%lld,\"imu_late_start_ms\":%lld",
                       (unsigned long long)st_fused_sent, (long long)s_imu_t0_run, (long long)s_imu_late_start_ms);
    if (s_c.shape_every > 0)
        en += snprintf(extra + en, sizeof(extra) - en, ",\"shape_sent\":%llu,\"shape_skip\":%llu",
                       (unsigned long long)st_shape_sent, (unsigned long long)st_shape_skip);
    if (s_c.feed_range)
        en += snprintf(extra + en, sizeof(extra) - en, ",\"range_fed\":%llu,\"range_skip\":%llu,\"range_rej\":%llu",
                       (unsigned long long)st_rf_fed, (unsigned long long)st_rf_skip, (unsigned long long)st_rf_rej);
    if (s_c.feed_imu)
        en += snprintf(extra + en, sizeof(extra) - en,
                       ",\"imu_pushed\":%llu,\"imu_late\":%llu,\"imu_early\":%llu,\"imu_lost\":%llu,\"imu_wait_to\":%llu",
                       (unsigned long long)st_imu_pushed, (unsigned long long)st_imu_late,
                       (unsigned long long)st_imu_early, (unsigned long long)st_imu_lost,
                       (unsigned long long)st_imu_wait_to);
    if (s_c.replay_log) /* 2026-11-15: 送らなかった再生の記録 (0 でなければ記録は不完全) */
        en += snprintf(extra + en, sizeof(extra) - en, ",\"replay_skip\":%llu", (unsigned long long)st_rl_skip);
    (void)en;
    dyne_mqtt_pub_status(DYNE_ST_DONE, extra);
    ESP_LOGI(TAG, "BSP(v2) done: crc=0x%08X", (unsigned)crc);
}
