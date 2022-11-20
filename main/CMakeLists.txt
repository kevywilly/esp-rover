set(SOURCES app_main.cpp vl53l0x.cpp)
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        )