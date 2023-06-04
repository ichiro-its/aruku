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

set(protobuf_MODULE_COMPATIBLE TRUE)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
if(CMAKE_CROSSCOMPILING)
  find_program(_PROTOBUF_PROTOC protoc)
else()
  set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
endif()

# Find gRPC installation
# Looks for gRPCConfig.cmake file installed by gRPC's cmake installation.
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

set(_GRPC_GRPCPP gRPC::grpc++)
if(CMAKE_CROSSCOMPILING)
  find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
else()
  set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
endif()

# Proto file
get_filename_component(aruku_proto "../aruku_interfaces/proto/aruku.proto" ABSOLUTE)
get_filename_component(aruku_proto_path "${aruku_proto}" PATH)

# Generated sources
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto")
set(aruku_proto_srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto/aruku.pb.cc")
set(aruku_proto_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto/aruku.pb.h")
set(aruku_grpc_srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto/aruku.grpc.pb.cc")
set(aruku_grpc_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto/aruku.grpc.pb.h")
add_custom_command(
      OUTPUT "${aruku_proto_srcs}" "${aruku_proto_hdrs}" "${aruku_grpc_srcs}" "${aruku_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto"
        --cpp_out "${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto"
        -I "${aruku_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${aruku_proto}"
      DEPENDS "${aruku_proto}")

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/config/node/config_node.cpp"
  "src/${PROJECT_NAME}/config/utils/config.cpp"
  "src/${PROJECT_NAME}/node/aruku_node.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_manager.cpp"
  "src/${PROJECT_NAME}/walking/node/walking_node.cpp"
  "src/${PROJECT_NAME}/walking/process/kinematic.cpp"
  "src/${PROJECT_NAME}/grpc/node/grpc_node.cpp"
  "${aruku_proto_srcs}"
  "${aruku_proto_hdrs}"
  "${aruku_grpc_srcs}"
  "${aruku_grpc_hdrs}"
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}/proto>
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
  rclcpp
  gRPC
  )

target_link_libraries(${PROJECT_NAME}
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
)


install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(config "src/config_main.cpp")
target_include_directories(config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(config ${PROJECT_NAME})

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

add_executable(grpc "src/grpc_main.cpp")
target_include_directories(grpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(grpc ${PROJECT_NAME})

install(TARGETS
  config
  control
  main
  walking
  grpc
  DESTINATION lib/${PROJECT_NAME})

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
  tachimawari_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
