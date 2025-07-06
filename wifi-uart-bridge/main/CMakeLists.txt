idf_component_register(SRC_DIRS "." "wifi" "server" "uart"
    PRIV_REQUIRES esp_driver_uart spi_flash esp_wifi nvs_flash
                    INCLUDE_DIRS "." "wifi" "server" "uart")
