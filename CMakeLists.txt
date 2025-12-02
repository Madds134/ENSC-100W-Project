idf_component_register(
    SRCS
        "esp_ibeacon_api.c"
        "ibeacon_demo.c"
        "stepper/stepper.c"
        "i2c/i2c_bus.c"
        "vl53l0x/vl53l0x.c"
        # "wifi/wifi.c"
    PRIV_REQUIRES
        driver freertos bt nvs_flash
    INCLUDE_DIRS
        "."
        "stepper"
        "i2c"
        "vl53l0x"
)
