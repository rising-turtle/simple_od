project(simple_od)

find_package(OpenCV REQUIRED)

find_package(glfw3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
# message("STATUS OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}")

# add libraries: glfw and librealsense
include_directories(/usr/local/include /home/hannah/work/librealsense/include)
link_directories(/usr/local/lib /home/hannah/work/librealsense/build)


add_executable(match_example match_example.cpp)
target_link_libraries(match_example ${OpenCV_LIBS})


add_executable(test_detector test_detector.cpp detector.cpp)
target_link_libraries(test_detector ${OpenCV_LIBS})

add_executable(test_tracker test_tracker.cpp tracker.cpp)
target_link_libraries(test_tracker ${OpenCV_LIBS})


# test object detection on rs200's data
add_executable(test_rs200 test_rs200.cpp)
target_link_libraries(test_rs200 glfw realsense GLU GL ${OpenCV_LIBS})


