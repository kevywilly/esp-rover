set(SOURCES
        app_main.cpp
        src/motor/servo.cpp
        src/app/app_httpd.cpp
        src/app/app_look.cpp
        src/app/app_auto_drive.cpp
        src/app/app_drive.cpp
        src/app/app_proximity.cpp
        src/sensor/vl53l0x.cpp
        src/sensor/tofarray.cpp

        src/robot.cpp
        )
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        "./include/app"
        "./include/motor"
        "./include/sensor"
        "./include/web"
        "./include/util"
        )