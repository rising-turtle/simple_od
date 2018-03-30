project(simple_od)

find_package(OpenCV REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
# message("STATUS OpenCV_INCLUDE_DIRS : ${OpenCV_INCLUDE_DIRS}")

# add libraries: glfw and librealsense
include_directories(/usr/local/include /home/hannah/work/librealsense/include)
link_directories(/usr/local/lib /home/hannah/work/librealsense/build)

# add library fast 
find_package(fast REQUIRED
	PATH /home/hannah/work/github/fast/build)
include_directories(${fast_INCLUDE_DIR})

## compile into a library 
add_library(simple_od global.cpp detector.cpp tracker.cpp detector_color.cpp)
target_link_libraries(simple_od ${OpenCV_LIBS})


add_executable(match_example match_example.cpp)
target_link_libraries(match_example ${OpenCV_LIBS})


add_executable(test_detector test_detector.cpp )
target_link_libraries(test_detector ${OpenCV_LIBS} simple_od)

add_executable(test_tracker test_tracker.cpp )
target_link_libraries(test_tracker ${OpenCV_LIBS} simple_od)

## test object detection on rs200's data
add_executable(test_rs200 test_rs200.cpp)
target_link_libraries(test_rs200 glfw realsense GLU GL ${OpenCV_LIBS})

add_executable(main_od main_od.cpp)
target_link_libraries(main_od simple_od realsense)

add_executable(test_fast test_fast.cpp)
target_link_libraries(test_fast realsense ${fast_LIBRARY} simple_od)

add_executable(main_od_color main_od_color.cpp)
target_link_libraries(main_od_color realsense simple_od)

## some other test 
add_executable(test_hsv test_hsv_value.cpp)
target_link_libraries(test_hsv ${OpenCV_LIBS})
