cmake_minimum_required(VERSION 3.8)
project(lucid_raw_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LucidMetadata.msg"
  DEPENDENCIES std_msgs
)

# ---- Arena SDK ----
# User install without the SDK's sudo ld.so.conf step: take the root from ARENA_ROOT (scripts/
# setup_lucid_env.bash exports it), else ~/ArenaSDK_Linux_x64. The libraries are found at run time
# through DT_RPATH (below), so neither LD_LIBRARY_PATH nor /etc/ld.so.conf.d is needed — DM_clipGUI
# clears LD_LIBRARY_PATH before it launches a sensor group.
set(ARENA_ROOT "$ENV{ARENA_ROOT}" CACHE PATH "Arena SDK root directory")
if(NOT ARENA_ROOT)
  set(ARENA_ROOT "$ENV{HOME}/ArenaSDK_Linux_x64" CACHE PATH "Arena SDK root directory" FORCE)
endif()
set(ARENA_GENICAM_LIB_DIR "${ARENA_ROOT}/GenICam/library/lib/Linux64_x64")
find_path(ARENA_INCLUDE_DIR ArenaApi.h HINTS "${ARENA_ROOT}/include/Arena" NO_DEFAULT_PATH)
find_path(ARENA_GENICAM_INCLUDE_DIR GenICam.h HINTS "${ARENA_ROOT}/GenICam/library/CPP/include" NO_DEFAULT_PATH)
find_library(ARENA_LIBRARY arena HINTS "${ARENA_ROOT}/lib64" NO_DEFAULT_PATH)
find_library(ARENA_GCBASE_LIBRARY GCBase_gcc54_v3_3_LUCID HINTS "${ARENA_GENICAM_LIB_DIR}" NO_DEFAULT_PATH)
find_library(ARENA_GENAPI_LIBRARY GenApi_gcc54_v3_3_LUCID HINTS "${ARENA_GENICAM_LIB_DIR}" NO_DEFAULT_PATH)
if(NOT ARENA_INCLUDE_DIR OR NOT ARENA_GENICAM_INCLUDE_DIR OR NOT ARENA_LIBRARY OR NOT ARENA_GCBASE_LIBRARY
    OR NOT ARENA_GENAPI_LIBRARY)
  message(FATAL_ERROR "Arena SDK not found under ARENA_ROOT='${ARENA_ROOT}'. Set ARENA_ROOT.")
endif()
message(STATUS "Arena SDK: ${ARENA_ROOT}")

add_executable(lucid_raw_camera_node
  src/lucid_raw_camera_node.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/build_info.cpp
)
add_custom_target(lucid_build_info
  COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DOUT=${CMAKE_CURRENT_BINARY_DIR}/build_info.cpp -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/build_info.cmake
  BYPRODUCTS ${CMAKE_CURRENT_BINARY_DIR}/build_info.cpp
  COMMENT "build_info.cpp (git describe)")
add_dependencies(lucid_raw_camera_node lucid_build_info)

target_compile_features(lucid_raw_camera_node PUBLIC cxx_std_17)
target_include_directories(lucid_raw_camera_node SYSTEM PRIVATE
  ${ARENA_INCLUDE_DIR}
  ${ARENA_GENICAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(lucid_raw_camera_node
  ${ARENA_LIBRARY}
  ${ARENA_GCBASE_LIBRARY}
  ${ARENA_GENAPI_LIBRARY}
  ${OpenCV_LIBS}
  Threads::Threads
)
ament_target_dependencies(lucid_raw_camera_node rclcpp sensor_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(lucid_raw_camera_node ${cpp_typesupport_target})

# DT_RPATH (not RUNPATH): also searched for libarena's own dependencies (libgentl, GenApi, …),
# which carry no RPATH of their own.
# libgentl needs the Metavision (event camera) libraries even for area-scan cameras.
set(_arena_rpath "${ARENA_ROOT}/lib64;${ARENA_GENICAM_LIB_DIR};${ARENA_ROOT}/ffmpeg;${ARENA_ROOT}/Metavision/lib")
string(REPLACE ";" ":" _arena_rpath_link "${_arena_rpath}")
set_target_properties(lucid_raw_camera_node PROPERTIES
  BUILD_RPATH "${_arena_rpath}"
  INSTALL_RPATH "${_arena_rpath}"
  LINK_FLAGS "-Wl,--disable-new-dtags -Wl,-rpath-link,${_arena_rpath_link}"
)

install(TARGETS lucid_raw_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
