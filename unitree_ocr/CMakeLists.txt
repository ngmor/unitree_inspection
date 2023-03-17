cmake_minimum_required(VERSION 3.8)
project(unitree_ocr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(unitree_ocr_interfaces REQUIRED)

include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
)

# OCR library
add_library(unitree_ocr src/ocr.cpp)
target_link_libraries(unitree_ocr ${OpenCV_LIBRARIES})

target_include_directories(unitree_ocr
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# text detection subscriber node
add_executable(text_detection_subscriber src/text_detection_subscriber.cpp)
ament_target_dependencies(text_detection_subscriber
  rclcpp std_msgs sensor_msgs cv_bridge image_transport unitree_ocr_interfaces
)
target_link_libraries(text_detection_subscriber
  ${OpenCV_LIBRARIES} unitree_ocr
)

install(TARGETS
  text_detection_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# install include directories
install(DIRECTORY include/ DESTINATION include)

# install targets
install(TARGETS unitree_ocr EXPORT unitree_ocr-targets)

# export targets
install(EXPORT unitree_ocr-targets
        FILE unitree_ocr-config.cmake
        NAMESPACE unitree_ocr::
        DESTINATION lib/cmake/${PROJECT_NAME})

install(DIRECTORY
  launch models
  DESTINATION share/${PROJECT_NAME}
)

find_package(Doxygen)
doxygen_add_docs(doxygen include/unitree_ocr/ocr.hpp src/ocr.cpp ALL)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
