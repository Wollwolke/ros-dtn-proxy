cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(dtn_proxy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_babel_fish REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

file(GLOB SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ros/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline/*.cpp)

add_executable(dtnproxy ${SOURCES})
ament_target_dependencies(dtnproxy rclcpp geometry_msgs sensor_msgs tf2 tf2_ros ros2_babel_fish)
target_compile_definitions(dtnproxy PRIVATE ASIO_STANDALONE)

target_include_directories(dtnproxy PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/cpp-httplib
  ${CMAKE_CURRENT_SOURCE_DIR}/external/toml11
  ${CMAKE_CURRENT_SOURCE_DIR}/external/websocketpp)

# qoixx
add_library(qoixx INTERFACE)
target_include_directories(qoixx INTERFACE external/qoixx/include)
target_link_libraries(dtnproxy qoixx)

# data-compressor
add_custom_target(dataCompressor
  COMMAND $(MAKE) -C ${CMAKE_CURRENT_SOURCE_DIR}/external/data-compressor/DataCompressor/DCLib/build/gcc/
)
add_dependencies(dtnproxy dataCompressor)

target_include_directories(dtnproxy PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/data-compressor/DataCompressor/DCLib/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/data-compressor/DataCompressor/DCIOLib/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/external/data-compressor/DataCompressor/common/inc)

target_link_libraries(
  dtnproxy
  ${CMAKE_CURRENT_SOURCE_DIR}/external/data-compressor/DataCompressor/build/gcc/DCLib.a
)

# nlohmann json
set(JSON_Install OFF CACHE INTERNAL "")
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/json)
target_link_libraries(dtnproxy nlohmann_json::nlohmann_json)

install(TARGETS
  dtnproxy
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
