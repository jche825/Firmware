px4_add_module(
        MODULE drivers__arduino_tacho
        MAIN ardutacho
        COMPILE_FLAGS
        SRCS
            ardutacho.cpp
            ArduinoTacho.cpp
        DEPENDS
    )