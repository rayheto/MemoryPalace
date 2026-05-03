add_executable(c_oop
    main.c
    bldc.c
    stepper.c
)

target_include_directories(c_oop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
