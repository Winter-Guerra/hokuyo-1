add_definitions(-std=gnu99)

include_directories(${PROJECT_SOURCE_DIR}/src/hokuyo/liburg/include
    ${PROJECT_SOURCE_DIR}/src/hokuyo/liburg/samples
    ${LCM_INCLUDE_DIRS})

add_executable(${POD_NAME}
    hokuyo.c
    liburg/samples/open_urg_sensor.c
    liburg/urg_connection.c
    liburg/urg_debug.c
    liburg/urg_ring_buffer.c
    liburg/urg_sensor.c
    liburg/urg_serial.c
    liburg/urg_serial_utils.c
    liburg/urg_tcpclient.c
    liburg/urg_utils.c
    )

pods_use_pkg_config_packages(${POD_NAME}
    bot2-core lcm)

target_link_libraries(${POD_NAME}
    ${LCM_LDFLAGS})

pods_install_executables(${POD_NAME})
