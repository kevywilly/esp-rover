set(SOURCES
        app_main.cpp
        src/vl53l0x.cpp
        src/servo.cpp
        src/app_vision.cpp
        src/app_auto_drive.cpp
        src/app_drive.cpp
        )
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        )