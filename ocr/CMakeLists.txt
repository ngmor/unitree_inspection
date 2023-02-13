cmake_minimum_required(VERSION 3.9)
project(ocr)


# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/opencv2/install/lib/cmake")



find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(text_detection src/text_detection.cpp)
target_link_libraries(text_detection ${OpenCV_LIBRARIES})

# enable C++ 17
target_compile_features(text_detection PUBLIC cxx_std_17) 

target_compile_options(text_detection PUBLIC -Wall -Wextra -Wpedantic)