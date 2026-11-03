idf_component_register(
    SRCS "main.c"
         "ble_scan_model.c"
         "ble_scan.c"
         "ble_scan_ui.c"
         "md5.c"
         "device_proto.c"
         "nova_proto.c"
         "framed_proto.c"
         "aura_proto.c"
         "seeker_proto.c"
         "ble_client.c"
    INCLUDE_DIRS "."
    REQUIRES bsp bt esp_event esp_hw_support esp_netif esp_timer esp_wifi nvs_flash
)

# 应用专用中文字体子集由 assets/fonts/app_font_16.c 提供(生成方式见该目录 README)。
# 生成文件不放进 main/,避免和手写源码混在一起;这里只把它编进 main 组件。
target_sources(${COMPONENT_LIB} PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../assets/fonts/app_font_16.c"
)
