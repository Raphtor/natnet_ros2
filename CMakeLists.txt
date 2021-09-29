cmake_minimum_required(VERSION 3.5)
project(natnet)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find_library(LIBNATNET NAMES NatNet PATHS lib)
add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp geometry_msgs)


add_library(NatNet SHARED IMPORTED)

set_property(TARGET NatNet PROPERTY
             IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libNatNet.so")
      
target_link_libraries(client NatNet)

install(TARGETS
  client
  
  DESTINATION lib/${PROJECT_NAME})


install(FILES 
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libNatNet.so"
  DESTINATION lib)
# Install the shared library -- does this need to be this complicated?
if(WIN32)
    ament_environment_hooks(env_hook/natnet_library_path.bat)
    set(ENV_VAR_NAME "PATH")
    set(ENV_VAR_VALUE "opt\\natnet\\bin")
else()
  ament_environment_hooks(env_hook/natnet_library_path.sh)
  if(APPLE)
    set(ENV_VAR_NAME "DYLD_LIBRARY_PATH")
  else()
    set(ENV_VAR_NAME "LD_LIBRARY_PATH")
  endif()
  set(ENV_VAR_VALUE "opt/natnet/lib")
endif()
ament_environment_hooks(env_hook/natnet_library_path.dsv.in)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
