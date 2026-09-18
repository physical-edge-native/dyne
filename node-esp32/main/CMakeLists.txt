idf_component_register(
    SRCS "main.c" "dyne_mqtt.c" "sensors.c" "rssi.c"
    INCLUDE_DIRS "."
    EMBED_FILES "heat1d_actor_small.wasm"
    PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer nvs_flash mqtt json esp_driver_i2c
)
