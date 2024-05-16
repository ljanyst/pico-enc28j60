pico_add_library(pico_enc28j60_driver)

pico_generate_pio_header(pico_enc28j60_driver
  ${CMAKE_CURRENT_LIST_DIR}/enc28j60-spi.pio
)

target_sources(pico_enc28j60_driver INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/enc28j60-driver.c
  ${CMAKE_CURRENT_LIST_DIR}/enc28j60-cmd.c
)

target_include_directories(pico_enc28j60_driver_headers INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(
  pico_enc28j60_driver
  INTERFACE
  hardware_pio
)

pico_add_library(pico_enc28j60_freertos_driver)

target_sources(pico_enc28j60_freertos_driver INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/enc28j60-freertos-driver.c
)

target_include_directories(pico_enc28j60_freertos_driver_headers INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(
  pico_enc28j60_freertos_driver
  INTERFACE
  freertos_plus_tcp
  pico_enc28j60_driver
)
