

project(motor_control)

link_directories("/usr/lib/x86_64-linux-gnu/")

add_library(motor control.cpp motor.cpp)
target_link_libraries(motor boost_unit_test_framework serial)

add_executable(test_control test_control.cpp)
target_link_libraries(test_control motor)
