cmake_minimum_required(VERSION 3.5)
project(aruku)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tachimawari REQUIRED)
find_package(framework_common REQUIRED)
find_package(framework_math REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME}
  "src/${PROJECT_NAME}/walking.cpp"
  "src/${PROJECT_NAME}/imu_filter.cpp"
  "src/${PROJECT_NAME}/imu.cpp"
  "src/${PROJECT_NAME}/stateless_orientation.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  tachimawari
  framework_math
  framework_common
  geometry_msgs
  tf2)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  tachimawari
  framework_math
  framework_common
  geometry_msgs
  tf2)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
