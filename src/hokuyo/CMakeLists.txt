add_definitions(-g)

include_directories(${PROJECT_SOURCE_DIR}/src/hokuyo/liburg/include/c
    ${PROJECT_SOURCE_DIR}/src/hokuyo/liburg/include/cpp
    ${PROJECT_SOURCE_DIR}/src/hokuyo/liburg/samples
    ${LCM_INCLUDE_DIRS})

add_executable(${POD_NAME}
    hokuyo.cpp
    liburg/urg_connection.c
    liburg/urg_debug.c
    liburg/Urg_driver.cpp
    liburg/urg_ring_buffer.c
    liburg/urg_sensor.c
    liburg/urg_serial.c
    liburg/urg_serial_utils.c
    liburg/urg_tcpclient.c
    liburg/urg_utils.c
    liburg/ticks.cpp
    liburg/samples/Connection_information.cpp
    liburg/samples/get_distance.cpp
    liburg/samples/get_distance_intensity.cpp
    liburg/samples/get_multiecho.cpp
    liburg/samples/get_multiecho_intensity.cpp
    liburg/samples/sensor_parameter.cpp
    liburg/samples/sync_time_stamp.cpp
    )

pods_use_pkg_config_packages(${POD_NAME}
    bot2-core lcm lcmtypes)

target_link_libraries(${POD_NAME}
    ${LCM_LDFLAGS})

pods_install_executables(${POD_NAME})
