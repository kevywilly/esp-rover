set(SOURCES
        src/esplidar.cpp
        src/espserial.cpp

        )
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        "./rplidar_sdk/sdk/include"
        )