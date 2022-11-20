set(SOURCES app_main.cpp src/tof_sensor.cpp)
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        )