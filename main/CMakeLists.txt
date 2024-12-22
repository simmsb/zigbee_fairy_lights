# idf_component_register(
#     SRC_DIRS  "."
#     INCLUDE_DIRS "."
#     PRIV_REQUIRES nvs_flash esp_driver_uart ieee802154
# )
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources})
