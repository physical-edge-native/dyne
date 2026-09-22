/*
 * Dyne node-esp32: 共通宣言 (発注7、2026-09-14)
 *
 * 全ノード同一ファームウェア。役割は起動時の設定注入で決まる (roadmap v3 4章)。
 * 制御面 = MQTT第1段 (規約: docs/2026-09-14-mqtt-control-plane.md)。
 * データ面 = 自前40バイト意味論 + UDP直送 (ワイヤ形式、M2から継続)。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "lwip/sockets.h"

/* ==== 定数 ==== */
#define DYNE_FW_NAME     "node-esp32 exp1"
#define DYNE_WIRE_LEN    40
#define DYNE_MAX_IFACES  2   /* 1次元チェーン: 左右の界面まで */
#define DYNE_SOE_MAX     64  /* transparent 指数和係数の上限 (dyne-core と一致) */

/* 境界条件コード (dyne-core::actor と同一) */
#define DYNE_BC_DIRICHLET     0
#define DYNE_BC_INTERFACE     1
#define DYNE_BC_ZERO_GRAD     2
#define DYNE_BC_LINEAR_EXTRAP 3
#define DYNE_BC_TRANSPARENT   4

/* ==== 設定 (dyne/cfg/<id> の受理結果) ==== */
typedef struct {
    bool valid;
    /* 世代番号 (規約 §3.6 鮮度管理。注入ツールが単調増加で付与、ackにエコー) */
    uint32_t gen;
    /* cluster */
    int n_nodes, points;
    double alpha, dt;
    uint32_t steps;
    uint32_t seed;      /* 現行の熱伝導アクターでは未使用 (規約 §2.1) */
    /* node */
    int idx, n;
    double x0, x1;
    /* bc */
    int bc_left, bc_right;
    double bc_val_left, bc_val_right;
    int t_m;                       /* transparent 係数の項数 (0 = 未注入) */
    double t_decay[DYNE_SOE_MAX], t_half[DYNE_SOE_MAX], t_weight[DYNE_SOE_MAX];
    /* ifaces (データ面の結線) */
    int n_ifaces;
    struct {
        int id;                    /* クラスタ一意の界面ID */
        int face;                  /* 自分側の面: 0=左, 1=右 */
        struct sockaddr_in peer;   /* UDP直送の相手 */
    } ifaces[DYNE_MAX_IFACES];
    /* mode */
    int role;                      /* 0=compute, 1=observe */
    int sensor;                    /* 0=none, 1=dht20, 2=bme280 */
    int sensor_period_ms, rssi_period_ms;
    int field_every, residual_every, rt_every;
    /* assim (発注8: 観測点ナッジング。規約 §2.1) */
    double nudge_gamma;            /* 0.0 = 無効 */
    int obs_period;                /* 観測周期 (ステップ。シミュレータと同一の意味論) */
    int obs_point;                 /* 観測点 = 担当区間内の局所格子点 (-1 = 中央 n/2) */
    /* 発注11 (実験1準備) */
    bool holdout;                  /* 保留ノード: 観測を同化に使わず (γ=0適用)
                                      イノベーションのみ記録・発行 (抜き取り検証) */
    double calib_offset;           /* センサ校正オフセット [℃]。読取値に加算してから
                                      同化・発行に使う (規約 §2.6: 生値も併載) */
    double init_value;             /* 初期場の一様値 (実測平均)。NAN = 従来のsin初期値 */
    int step_period_ms;            /* 1ステップの実時間周期 [ms]。0 = 従来 (最速) */
} dyne_cfg_t;

/* ==== ノード状態 (status トピックの state) ==== */
typedef enum {
    DYNE_ST_ONLINE,
    DYNE_ST_CONFIGURED,
    DYNE_ST_RUNNING,
    DYNE_ST_DONE,
    DYNE_ST_ERR,
} dyne_state_t;

/* ==== グローバル (main.c 所有) ==== */
extern dyne_cfg_t g_cfg;
extern volatile bool g_start_req;  /* ctl: start 受信 */
extern volatile bool g_stop_req;   /* ctl: stop 受信 */
extern volatile bool g_report_req; /* ctl: report 受信 */

int dyne_node_id(void);

/* main.c: MQTT層からのコールバック (制御面→ノードの入口) */
void dyne_on_cfg(const char *payload, int len);
void dyne_on_ctl(const char *payload, int len);
void dyne_publish_status_running(uint32_t step);

/* dyne_mqtt.c: MQTTクライアント層 (第2段=zenoh移行時はこの層だけ差し替える) */
void dyne_mqtt_start(void);
bool dyne_mqtt_connected(void);
/* subtopic 例: "node/3/field"。qos0 の観測系は enqueue=true (BSPループを止めない) */
void dyne_mqtt_pub(const char *subtopic, const char *json, int qos, bool retain,
                   bool enqueue);
void dyne_mqtt_pub_status(dyne_state_t st, const char *extra_json_fields);

/* sensors.c: DHT20 / BME280 (I2C)。読取はタスクが周期実行し sensor トピックへ発行 */
void dyne_sensors_start(void);
/* 最新の温度読取値 (発注8: ナッジングの観測値)。読取成功が一度もなければ false */
bool dyne_sensor_latest(double *temp_c, int64_t *t_ms);

/* rssi.c: AP RSSI + 近傍RSSI (プロミスキャス受信)。rssi トピックへ周期発行 */
void dyne_rssi_start(void);

/* ==== ワイヤ形式 (dyne-wire と同一レイアウト・40バイト) ==== */
uint32_t dyne_crc32(const uint8_t *d, size_t len);
void dyne_wire_encode(uint8_t *b, uint16_t iface, uint64_t step, uint32_t seq,
                      double value, double grad);
bool dyne_wire_decode(const uint8_t *b, int len, uint16_t *iface,
                      uint64_t *step, double *value, double *grad);
