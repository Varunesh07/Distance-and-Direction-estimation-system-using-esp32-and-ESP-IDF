idf_component_register(
    SRCS  "main.c" "webserver.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash  esp_wifi esp_event esp_netif esp_system freertos  bt esp_driver_gpio esp_driver_ledc esp_timer  esp_http_server
)
