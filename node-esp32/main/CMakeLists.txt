idf_component_register(
    SRCS "main.c" "dyne_mqtt.c" "sensors.c" "rssi.c" "dyne_wire2.c" "node_v2.c"
         "dyne_timesync_core.c" "dyne_timesync.c" "dyne_actor_slot.c" "dyne_ops.c"
         "dyne_sense_core.c" "dyne_range.c" "dyne_imu.c" "dyne_shape_msg.c"
         "dyne_node_id_core.c" "dyne_node_id.c" "dyne_sync_core.c"
    INCLUDE_DIRS "."
    EMBED_FILES "heat1d_actor_small.wasm"
    PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer nvs_flash mqtt json esp_driver_i2c
                 esp_driver_gpio app_update esp_https_ota esp_http_client esp_partition
                 esp_app_format mbedtls lwip esp_driver_ledc
)
