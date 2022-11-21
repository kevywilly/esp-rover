set(SOURCES app_main.cpp src/vl53l0x.cpp src/servo.cpp src/app_vision.cpp)
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        )