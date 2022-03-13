cmake_minimum_required(VERSION 3.5)
project(aruku)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_compILER_IS_GNUCXX OR CMAKE_CXX_compILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aruku_interfaces REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/config/node/config_node.cpp"
  "src/${PROJECT_NAME}/config/utils/config_util.cpp"
  "src/${PROJECT_NAME}/node/aruku_node.cpp"
  "src/${PROJECT_NAME}/walking/model/kinematic_id.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_manager.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_node.cpp"
  "src/${PROJECT_NAME}/walking/process/kinematic.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  aruku_interfaces
  kansei_interfaces
  keisan
  tachimawari
  tachimawari_interfaces)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(aruku_main "src/aruku_main.cpp")
target_include_directories(aruku_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(aruku_main ${PROJECT_NAME})

add_executable(config_main "src/config_main.cpp")
target_include_directories(config_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(config_main ${PROJECT_NAME})

install(TARGETS
  aruku_main
  config_main
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  aruku_interfaces
  kansei_interfaces
  keisan
  tachimawari
  tachimawari_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
