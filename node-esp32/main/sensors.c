/*
 * Dyne node-esp32: センサ読取 (DHT20 / BME280、I2C)
 *
 * 発注7(ii): 読取値は当面ログ発行のみ (dyne/node/<id>/sensor)。
 * データ同化への接続は発注8。切替は設定注入の mode.sensor (規約 §2.1)。
 *
 * - DHT20 (アドレス 0x38): 温湿度。測定トリガ 0xAC 0x33 0x00 → 85ms → 7バイト読み。
 *   CRC8 (多項式 0x31、初期値 0xFF) で検証。
 * - BME280 (アドレス 0x76、Kconfigで0x77へ変更可): 温湿圧。データシート 4.2.3 の
 *   整数補償式 (int32/int64) をそのまま実装。ノーマルモード・オーバーサンプル×1。
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "dyne_timesync.h" /* 2026-11-11: 取得時刻の同期した時刻 */

#include "dyne_node.h"

static const char *TAG = "dyne-sens";

#define DHT20_ADDR  0x38
#define BME280_ADDR CONFIG_DYNE_BME280_ADDR

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dht20, s_bme280;
static bool s_bme_ok;

/* ---- DHT20 ---- */

static uint8_t crc8_dht(const uint8_t *d, int n)
{
    uint8_t c = 0xFF;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int b = 0; b < 8; b++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
    }
    return c;
}

static bool dht20_read(double *temp_c, double *rh)
{
    const uint8_t trig[3] = { 0xAC, 0x33, 0x00 };
    if (i2c_master_transmit(s_dht20, trig, 3, 100) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(85));
    uint8_t d[7];
    if (i2c_master_receive(s_dht20, d, 7, 100) != ESP_OK) return false;
    if (d[0] & 0x80) return false;              /* まだ busy */
    if (crc8_dht(d, 6) != d[6]) return false;   /* CRC不一致 */
    uint32_t hraw = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
    uint32_t traw = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
    *rh = hraw * 100.0 / 1048576.0;
    *temp_c = traw * 200.0 / 1048576.0 - 50.0;
    return true;
}

/* ---- BME280 (データシートの補償式。t_fine を共有) ---- */

static struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;
} cal;
static int32_t t_fine;

static bool bme_rd(uint8_t reg, uint8_t *buf, int n)
{
    return i2c_master_transmit_receive(s_bme280, &reg, 1, buf, n, 100) == ESP_OK;
}
static bool bme_wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_bme280, b, 2, 100) == ESP_OK;
}

static bool bme280_init(void)
{
    uint8_t id = 0;
    if (!bme_rd(0xD0, &id, 1) || id != 0x60) {
        ESP_LOGW(TAG, "BME280 not found (id=0x%02X)", id);
        return false;
    }
    uint8_t c1[26], c2[7];
    if (!bme_rd(0x88, c1, 26) || !bme_rd(0xE1, c2, 7)) return false;
    cal.T1 = c1[0] | (c1[1] << 8);
    cal.T2 = (int16_t)(c1[2] | (c1[3] << 8));
    cal.T3 = (int16_t)(c1[4] | (c1[5] << 8));
    cal.P1 = c1[6] | (c1[7] << 8);
    cal.P2 = (int16_t)(c1[8] | (c1[9] << 8));
    cal.P3 = (int16_t)(c1[10] | (c1[11] << 8));
    cal.P4 = (int16_t)(c1[12] | (c1[13] << 8));
    cal.P5 = (int16_t)(c1[14] | (c1[15] << 8));
    cal.P6 = (int16_t)(c1[16] | (c1[17] << 8));
    cal.P7 = (int16_t)(c1[18] | (c1[19] << 8));
    cal.P8 = (int16_t)(c1[20] | (c1[21] << 8));
    cal.P9 = (int16_t)(c1[22] | (c1[23] << 8));
    cal.H1 = c1[25];
    cal.H2 = (int16_t)(c2[0] | (c2[1] << 8));
    cal.H3 = c2[2];
    cal.H4 = (int16_t)((c2[3] << 4) | (c2[4] & 0x0F));
    cal.H5 = (int16_t)((c2[5] << 4) | (c2[4] >> 4));
    cal.H6 = (int8_t)c2[6];
    /* 湿度×1 → 温度×1・気圧×1・ノーマルモード → 待機1000ms */
    if (!bme_wr(0xF2, 0x01)) return false;
    if (!bme_wr(0xF4, (1 << 5) | (1 << 2) | 3)) return false;
    if (!bme_wr(0xF5, 5 << 5)) return false;
    return true;
}

static int32_t bme_comp_t(int32_t adc)
{
    int32_t v1 = ((((adc >> 3) - ((int32_t)cal.T1 << 1))) * (int32_t)cal.T2) >> 11;
    int32_t v2 = (((((adc >> 4) - (int32_t)cal.T1) * ((adc >> 4) - (int32_t)cal.T1)) >> 12)
                  * (int32_t)cal.T3) >> 14;
    t_fine = v1 + v2;
    return (t_fine * 5 + 128) >> 8; /* 0.01°C 単位 */
}

static uint32_t bme_comp_p(int32_t adc)
{
    int64_t v1 = (int64_t)t_fine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)cal.P6;
    v2 += (v1 * (int64_t)cal.P5) << 17;
    v2 += ((int64_t)cal.P4) << 35;
    v1 = ((v1 * v1 * (int64_t)cal.P3) >> 8) + ((v1 * (int64_t)cal.P2) << 12);
    v1 = ((((int64_t)1 << 47) + v1) * (int64_t)cal.P1) >> 33;
    if (v1 == 0) return 0;
    int64_t p = 1048576 - adc;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)cal.P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)cal.P8 * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)cal.P7) << 4);
    return (uint32_t)p; /* Q24.8 Pa */
}

static uint32_t bme_comp_h(int32_t adc)
{
    int32_t v = t_fine - 76800;
    v = ((((adc << 14) - (((int32_t)cal.H4) << 20) - ((int32_t)cal.H5 * v)) + 16384) >> 15)
        * (((((((v * (int32_t)cal.H6) >> 10) * (((v * (int32_t)cal.H3) >> 11) + 32768)) >> 10)
             + 2097152) * (int32_t)cal.H2 + 8192) >> 14);
    v -= (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)cal.H1) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12); /* Q22.10 %RH */
}

static bool bme280_read(double *temp_c, double *rh, double *pres_hpa)
{
    uint8_t d[8];
    if (!bme_rd(0xF7, d, 8)) return false;
    int32_t adc_p = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adc_t = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    int32_t adc_h = ((int32_t)d[6] << 8) | d[7];
    *temp_c = bme_comp_t(adc_t) / 100.0;
    *pres_hpa = bme_comp_p(adc_p) / 256.0 / 100.0;
    *rh = bme_comp_h(adc_h) / 1024.0;
    return true;
}

/* ---- 最新読取値 (発注8: ナッジングの観測値としてBSPループが読む) ---- */

static portMUX_TYPE s_lat_mux = portMUX_INITIALIZER_UNLOCKED;
static struct { double temp_c; int64_t t_ms; bool valid; } s_latest;

static void latest_store(double temp_c, int64_t t_ms)
{
    portENTER_CRITICAL(&s_lat_mux);
    s_latest.temp_c = temp_c;
    s_latest.t_ms = t_ms;
    s_latest.valid = true;
    portEXIT_CRITICAL(&s_lat_mux);
}

bool dyne_sensor_latest(double *temp_c, int64_t *t_ms)
{
    portENTER_CRITICAL(&s_lat_mux);
    bool v = s_latest.valid;
    if (v) {
        *temp_c = s_latest.temp_c;
        *t_ms = s_latest.t_ms;
    }
    portEXIT_CRITICAL(&s_lat_mux);
    return v;
}

/* ---- 周期タスク ---- */

static void sensors_task(void *arg)
{
    for (;;) {
        int period = g_cfg.valid ? g_cfg.sensor_period_ms : 5000;
        int kind = g_cfg.valid ? g_cfg.sensor : 0;
        if (period <= 0 || kind == 0 || !dyne_mqtt_connected()) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        char sub[32], json[192];
        snprintf(sub, sizeof(sub), "node/%d/sensor", dyne_node_id());
        int64_t t_local = esp_timer_get_time();
        int64_t t_ms = t_local / 1000;
        /* 2026-11-11: 取得時刻を同期した時刻 (1970年からの µs、PC の時刻サーバが基準) でも付ける。
         * 未同期は 0。t_ms (起動からの ms) は従来どおり残す */
        long long ts_us = (long long)dyne_time_at_local(t_local);
        /* 発注11: 校正オフセット。補正後値 temp_c を同化・発行の正とし、
         * 生値 temp_c_raw も併載する (規約 §2.6) */
        double off = g_cfg.valid ? g_cfg.calib_offset : 0.0;
        if (kind == 1) {
            double t, h;
            if (dht20_read(&t, &h)) {
                latest_store(t + off, t_ms);
                snprintf(json, sizeof(json),
                         "{\"t_ms\":%lld,\"ts_us\":%lld,\"kind\":\"dht20\",\"temp_c\":%.2f,"
                         "\"temp_c_raw\":%.2f,\"rh\":%.2f}",
                         (long long)t_ms, ts_us, t + off, t, h);
            } else {
                snprintf(json, sizeof(json),
                         "{\"t_ms\":%lld,\"ts_us\":%lld,\"kind\":\"dht20\",\"err\":\"i2c read\"}",
                         (long long)t_ms, ts_us);
            }
        } else {
            double t, h, p;
            if (s_bme_ok && bme280_read(&t, &h, &p)) {
                latest_store(t + off, t_ms);
                snprintf(json, sizeof(json),
                         "{\"t_ms\":%lld,\"ts_us\":%lld,\"kind\":\"bme280\",\"temp_c\":%.2f,"
                         "\"temp_c_raw\":%.2f,\"rh\":%.2f,\"pres_hpa\":%.2f}",
                         (long long)t_ms, ts_us, t + off, t, h, p);
            } else {
                if (!s_bme_ok) s_bme_ok = bme280_init(); /* 後差しに追随 */
                snprintf(json, sizeof(json),
                         "{\"t_ms\":%lld,\"ts_us\":%lld,\"kind\":\"bme280\",\"err\":\"i2c read\"}",
                         (long long)t_ms, ts_us);
            }
        }
        dyne_mqtt_pub(sub, json, 0, false, true);
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}

/* 2026-11-13: I2C のバスを測距 (dyne_range.c)・IMU (dyne_imu.c) と共有する。最初に呼んだ所で作る
 * (起動の順は node_task の1本の流れなので排他は要らない) */
i2c_master_bus_handle_t dyne_i2c_bus(void)
{
    if (s_bus) return s_bus;
    i2c_master_bus_config_t bus = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .sda_io_num = CONFIG_DYNE_I2C_SDA,
        .scl_io_num = CONFIG_DYNE_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* 短距離・モジュール内蔵プルアップ前提の保険 */
    };
    if (i2c_new_master_bus(&bus, &s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed (sensor disabled)");
        s_bus = NULL;
    }
    return s_bus;
}

void dyne_sensors_start(void)
{
    if (!dyne_i2c_bus()) return;
    i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                               .scl_speed_hz = 100000 };
    dc.device_address = DHT20_ADDR;
    i2c_master_bus_add_device(s_bus, &dc, &s_dht20);
    dc.device_address = BME280_ADDR;
    i2c_master_bus_add_device(s_bus, &dc, &s_bme280);
    s_bme_ok = bme280_init(); /* 未接続なら失敗のまま (読取時に err 発行) */
    xTaskCreate(sensors_task, "dyne_sens", 4096, NULL, 3, NULL);
}
