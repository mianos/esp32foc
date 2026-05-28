set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(
    SRCS
        "main.cpp"
        "motor.cpp"
        "current_sense.cpp"
        "Settings.cpp"
        "WifiConnection.cpp"
        "MotorWebServer.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_mcpwm
        esp_adc
        esp_event
        esp_netif
        esp_http_server
        app_update
        nvsstoragemanager
        wifimanager
        webserver
        jsonwrapper
)
