cmake_minimum_required(VERSION 3.5)
project(aruku)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aruku_interfaces REQUIRED)
find_package(kansei REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tachimawari REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/config/node/config_node.cpp"
  "src/${PROJECT_NAME}/config/utils/config.cpp"
  "src/${PROJECT_NAME}/node/aruku_node.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_manager.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_node.cpp"
  "src/${PROJECT_NAME}/walking/process/kinematic.cpp"
  "src/${PROJECT_NAME}/config/grpc/config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_base.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_get_config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_set_config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_save_config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_publish_config.cpp"
  "src/${PROJECT_NAME}/config/grpc/call_data_set_app_status.cpp"
)

add_library(${PROJECT_NAME}_exported SHARED
  "src/${PROJECT_NAME}/walking/node/walking_manager.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_node.cpp"
  "src/${PROJECT_NAME}/walking/process/kinematic.cpp"
)

target_include_directories(${PROJECT_NAME}_exported PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  aruku_interfaces
  kansei
  kansei_interfaces
  keisan
  rclcpp
  tachimawari
  tachimawari_interfaces
  )

ament_target_dependencies(${PROJECT_NAME}_exported
  ament_index_cpp
  aruku_interfaces
  kansei
  kansei_interfaces
  keisan
  rclcpp
  tachimawari
  tachimawari_interfaces
  )

target_link_libraries(${PROJECT_NAME}
  gRPC::grpc++_reflection
  gRPC::grpc++
)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  )

install(TARGETS ${PROJECT_NAME}_exported
  EXPORT export_${PROJECT_NAME}_exported
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  )

target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

add_executable(control "src/control_main.cpp")
target_include_directories(control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(control ${PROJECT_NAME})

add_executable(main "src/aruku_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

add_executable(walking "src/walking_main.cpp")
target_include_directories(walking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(walking ${PROJECT_NAME})

install(TARGETS
  control
  main
  walking
  DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ament_index_cpp
  aruku_interfaces
  kansei
  kansei_interfaces
  keisan
  rclcpp
  tachimawari
  tachimawari_interfaces
  )
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME}_exported)
ament_package()
