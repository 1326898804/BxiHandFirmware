file(GLOB_RECURSE sources **.c **.cpp)
message(${sources})
idf_component_register(SRCS ${sources}
                       PRIV_REQUIRES spi_flash bt nvs_flash esp_driver_uart esp_driver_usb_serial_jtag esp_ringbuf
                       INCLUDE_DIRS "ftServo/include" "hand_driver"
                       )
