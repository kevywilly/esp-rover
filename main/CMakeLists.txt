set(SOURCES app_main.c)
idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS
        "."
        "./include"
        )