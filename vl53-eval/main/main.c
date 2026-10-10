/*
 * vl53-eval: VL53L5CX 性能評価用ファーム (docs/2026-10-10-vl53l5cx-eval-plan.md)
 *
 * 現役の node-esp32 とは独立した小さなプロジェクト。センサの測定値を
 * USBシリアル(921600bps)へ1行1フレームのCSVで流し、PC側の
 * scripts/vl53_log.py が記録する。設定は書き込み直さずにシリアルの
 * 1行コマンドで切り替える(停止中のみ):
 *
 *   res 4|8            区画数 (4x4 / 8x8)
 *   hz N               測定周期 [Hz] (8x8: 1〜15, 4x4: 1〜60)
 *   mode cont|auto     連続 / 自律 (積分時間 it は自律のときだけ効く)
 *   it N               積分時間 [ms]
 *   sharp N            シャープナー [%] (0〜99)
 *   order closest|strongest  1区画に複数物体があるときの優先
 *   start N            Nフレーム測って停止 (N=0 は stop まで)
 *   stop               停止
 *   info               現在の設定と起動情報を再表示
 *
 * 出力行 (先頭の文字で種類を区別):
 *   #...  情報・応答 (#BOOT / #INFO / #CFG / #OK / #ERR / #DONE)
 *   F,t_ms,idx,nz,temp_c, d[nz], st[nz], sg[nz], sp[nz], nt[nz], am[nz]
 *       d=距離[mm] st=状態値(5と9が有効) sg=ばらつき推定[mm]
 *       sp=信号の強さ[kcps/SPAD] nt=検出物体数 am=環境光[kcps/SPAD]
 *       区画の番号は ULD の並び (行優先、0番が先頭行の端。向きは評価E3で実測する)
 *
 * 配線 (SparkFun Qwiic Mini ToF Imager = スイッチサイエンス SKU 7934):
 *   3V3→3V3, GND→GND, SDA→GPIO21, SCL→GPIO22。INT・RSTは未接続
 *   (RSTは基板上で47kΩでGNDに落ちている=非リセット。LPnは端子なし=常時有効)。
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "vl53l5cx_api.h"

#define FW_NAME "vl53-eval v1"
#define CMD_LEN 64

static VL53L5CX_Configuration s_dev;   /* 約1.5KB+作業領域。スタックに置かない */
static VL53L5CX_ResultsData s_res;
static QueueHandle_t s_cmdq;
static char s_line[4096];
static uint8_t s_nz = 64;

static int64_t s_init_us;
static size_t s_heap_before, s_heap_after;

static void print_info(void)
{
    printf("#INFO fw=%s uld=%s i2c_hz=%d sda=%d scl=%d init_ms=%lld "
           "heap_free_before=%u heap_free_after=%u heap_min=%u largest=%u "
           "sizeof_dev=%u sizeof_res=%u\n",
           FW_NAME, VL53L5CX_API_REVISION, CONFIG_VL53E_I2C_HZ,
           CONFIG_VL53E_I2C_SDA, CONFIG_VL53E_I2C_SCL, s_init_us / 1000,
           (unsigned)s_heap_before, (unsigned)s_heap_after,
           (unsigned)esp_get_minimum_free_heap_size(),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
           (unsigned)sizeof(s_dev), (unsigned)sizeof(s_res));
}

static void print_cfg(void)
{
    uint8_t res = 0, hz = 0, sharp = 0, order = 0, mode = 0;
    uint32_t it = 0;
    vl53l5cx_get_resolution(&s_dev, &res);
    vl53l5cx_get_ranging_frequency_hz(&s_dev, &hz);
    vl53l5cx_get_integration_time_ms(&s_dev, &it);
    vl53l5cx_get_sharpener_percent(&s_dev, &sharp);
    vl53l5cx_get_target_order(&s_dev, &order);
    vl53l5cx_get_ranging_mode(&s_dev, &mode);
    printf("#CFG nz=%u hz=%u mode=%s it_ms=%u sharp=%u order=%s\n", res, hz,
           mode == VL53L5CX_RANGING_MODE_AUTONOMOUS ? "auto" : "cont",
           (unsigned)it, sharp,
           order == VL53L5CX_TARGET_ORDER_STRONGEST ? "strongest" : "closest");
}

/* シリアル1行受信タスク: 行をキューに積むだけ (測定ループを止めない) */
static void cmd_task(void *arg)
{
    char buf[CMD_LEN];
    for (;;) {
        if (fgets(buf, sizeof(buf), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        buf[strcspn(buf, "\r\n")] = 0;
        if (buf[0]) xQueueSend(s_cmdq, buf, portMAX_DELAY);
    }
}

static void reply(const char *cmd, uint8_t st)
{
    if (st == 0) printf("#OK %s\n", cmd);
    else printf("#ERR %s status=%u\n", cmd, st);
}

/* 戻り値: 測定を始めるフレーム数 (start 以外は -1) */
static long handle_cmd(char *cmd, bool ranging)
{
    char name[16] = { 0 }, arg[32] = { 0 };
    sscanf(cmd, "%15s %31s", name, arg);
    if (!strcmp(name, "info")) { print_info(); print_cfg(); return -1; }
    if (!strcmp(name, "stop")) {
        reply(cmd, ranging ? vl53l5cx_stop_ranging(&s_dev) : 0);
        return -2;
    }
    if (ranging) { printf("#ERR %s busy(stop first)\n", cmd); return -1; }
    uint8_t st = 255;
    if (!strcmp(name, "res")) {
        int v = atoi(arg);
        if (v == 4) st = vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_4X4);
        else if (v == 8) st = vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_8X8);
    } else if (!strcmp(name, "hz")) {
        st = vl53l5cx_set_ranging_frequency_hz(&s_dev, (uint8_t)atoi(arg));
    } else if (!strcmp(name, "mode")) {
        if (!strcmp(arg, "cont")) st = vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
        else if (!strcmp(arg, "auto")) st = vl53l5cx_set_ranging_mode(&s_dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);
    } else if (!strcmp(name, "it")) {
        st = vl53l5cx_set_integration_time_ms(&s_dev, (uint32_t)atoi(arg));
    } else if (!strcmp(name, "sharp")) {
        st = vl53l5cx_set_sharpener_percent(&s_dev, (uint8_t)atoi(arg));
    } else if (!strcmp(name, "order")) {
        if (!strcmp(arg, "closest")) st = vl53l5cx_set_target_order(&s_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
        else if (!strcmp(arg, "strongest")) st = vl53l5cx_set_target_order(&s_dev, VL53L5CX_TARGET_ORDER_STRONGEST);
    } else if (!strcmp(name, "start")) {
        /* 区画数は開始前に読んでおく (測定中のDCI読み出しは避ける) */
        vl53l5cx_get_resolution(&s_dev, &s_nz);
        print_cfg();
        st = vl53l5cx_start_ranging(&s_dev);
        reply(cmd, st);
        return st == 0 ? atol(arg) : -1;
    } else {
        printf("#ERR %s unknown\n", cmd);
        return -1;
    }
    reply(cmd, st);
    if (st == 0) print_cfg();
    return -1;
}

static void emit_frame(uint32_t idx, uint8_t nz)
{
    int n = snprintf(s_line, sizeof(s_line), "F,%lld,%u,%u,%d",
                     esp_timer_get_time() / 1000, (unsigned)idx, nz,
                     s_res.silicon_temp_degc);
#define APPEND(arr)                                                         \
    for (int i = 0; i < nz; i++)                                            \
        n += snprintf(s_line + n, sizeof(s_line) - n, ",%ld",              \
                      (long)s_res.arr[VL53L5CX_NB_TARGET_PER_ZONE * i]);
    APPEND(distance_mm)
    APPEND(target_status)
    APPEND(range_sigma_mm)
    APPEND(signal_per_spad)
#undef APPEND
    for (int i = 0; i < nz; i++)
        n += snprintf(s_line + n, sizeof(s_line) - n, ",%u", s_res.nb_target_detected[i]);
    for (int i = 0; i < nz; i++)
        n += snprintf(s_line + n, sizeof(s_line) - n, ",%lu", (unsigned long)s_res.ambient_per_spad[i]);
    s_line[n++] = '\n';
    fwrite(s_line, 1, n, stdout);
    fflush(stdout);
}

void app_main(void)
{
    /* stdin を UARTドライバ経由の行入力にする */
    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 512, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    setvbuf(stdin, NULL, _IONBF, 0);
    printf("#BOOT %s\n", FW_NAME);

    i2c_master_bus_config_t bus = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_VL53E_I2C_SDA,
        .scl_io_num = CONFIG_VL53E_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_h;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &bus_h));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1, /* 0x29 */
        .scl_speed_hz = CONFIG_VL53E_I2C_HZ,
    };
    s_dev.platform.bus_config = bus;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_h, &dev_cfg, &s_dev.platform.handle));
    /* RSTは未接続 (VL53L5CX_Reset_Sensor は呼ばない) */

    /* 同じバスにDHT20(0x38)が載っていても衝突しない。見えている番地を表示する */
    printf("#INFO i2c_scan=");
    for (uint16_t a = 0x08; a < 0x78; a++)
        if (i2c_master_probe(bus_h, a, 50) == ESP_OK) printf("0x%02X ", a);
    printf("\n");

    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&s_dev, &alive) || !alive) {
        printf("#ERR not_alive (配線・電源・番地0x29を確認)\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    s_heap_before = esp_get_free_heap_size();
    int64_t t0 = esp_timer_get_time();
    uint8_t st = vl53l5cx_init(&s_dev);   /* センサ用ファーム(約80KB)の書き込み */
    s_init_us = esp_timer_get_time() - t0;
    s_heap_after = esp_get_free_heap_size();
    if (st) {
        printf("#ERR init status=%u\n", st);
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    /* 評価の既定: 8x8・10Hz・連続 */
    vl53l5cx_set_resolution(&s_dev, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&s_dev, 10);
    print_info();
    print_cfg();

    s_cmdq = xQueueCreate(8, CMD_LEN);
    xTaskCreate(cmd_task, "cmd", 3072, NULL, 5, NULL);

    bool ranging = false;
    long target = 0;
    uint32_t idx = 0;
    char cmd[CMD_LEN];
    for (;;) {
        while (xQueueReceive(s_cmdq, cmd, 0) == pdTRUE) {
            long r = handle_cmd(cmd, ranging);
            if (r >= 0) { ranging = true; target = r; idx = 0; }
            else if (r == -2) ranging = false;
        }
        if (!ranging) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        uint8_t ready = 0;
        vl53l5cx_check_data_ready(&s_dev, &ready);
        if (!ready) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
        if (vl53l5cx_get_ranging_data(&s_dev, &s_res) == 0) emit_frame(idx++, s_nz);
        if (target > 0 && idx >= (uint32_t)target) {
            vl53l5cx_stop_ranging(&s_dev);
            ranging = false;
            printf("#DONE frames=%u\n", (unsigned)idx);
        }
    }
}
