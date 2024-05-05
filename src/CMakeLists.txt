if (NOT TARGET pico_enc28j60_driver)
  pico_add_library(pico_enc28j60_driver)

  pico_generate_pio_header(pico_enc28j60_driver
    ${CMAKE_CURRENT_LIST_DIR}/enc28j60_spi.pio
  )

  target_sources(pico_enc28j60_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/enc28j60_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/enc28j60_cmd.c
  )

  target_include_directories(pico_enc28j60_driver_headers INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
  )

  target_link_libraries(pico_malloc INTERFACE
    hardware_pio
  )
endif()
