project(simple_od)

find_package(OpenCV REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
message("STATUS OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}")

add_executable(match_example match_example.cpp)
target_link_libraries(match_example ${OpenCV_LIBS})


add_executable(test_detector test_detector.cpp detector.cpp)
target_link_libraries(test_detector ${OpenCV_LIBS})
