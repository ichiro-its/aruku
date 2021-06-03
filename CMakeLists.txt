cmake_minimum_required(VERSION 3.5)
project(aruku)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tachimawari REQUIRED)
find_package(robocup_client REQUIRED)
find_package(framework_common REQUIRED)
find_package(framework_math REQUIRED)
find_package(kansei REQUIRED)

add_library(${PROJECT_NAME}
  "src/${PROJECT_NAME}/walking.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  tachimawari
  robocup_client
  framework_math
  framework_common
  kansei)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(walking_main "src/walking_main.cpp")
target_include_directories(walking_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(walking_main ${PROJECT_NAME})

install(TARGETS
  walking_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  tachimawari
  robocup_client
  framework_math
  framework_common
  kansei)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
