idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver freertos nvs_flash esp_wifi esp_netif esp_event esp_timer
)
