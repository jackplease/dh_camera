cmake_minimum_required(VERSION 3.10)
project(dh_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/camera_api.cpp
  src/camera_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC GXSDK/inc)

# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC GXSDK/lib/x86_64)
  install(
    DIRECTORY GXSDK/lib/x86_64
    DESTINATION lib
  )
# elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#   target_link_directories(${PROJECT_NAME} PUBLIC GXSDK/lib/arm64)
#   install(
#     DIRECTORY hikSDK/lib/arm64/
#     DESTINATION lib
#   )
# else()
#   message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
# endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN dh_camera::DhCamera
  EXECUTABLE ${PROJECT_NAME}_node
)

# add_executable(dh_camera src/camera_node.cpp src/camera_api.cpp)
target_link_libraries(dh_camera /usr/lib/libgxiapi.so )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
