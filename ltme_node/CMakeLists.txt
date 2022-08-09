cmake_minimum_required(VERSION 3.8)
project(ltme_node)

# 启用c++ 14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated -Wno-error -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ltme_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies rclcpp ltme_interfaces sensor_msgs std_msgs rosidl_default_generators)

ament_package()

###########
## Build ##
###########

set(LDCP_SDK_DIR "lib/ldcp_sdk")
add_subdirectory(${LDCP_SDK_DIR})


include_directories(
  include
  "${PROJECT_SOURCE_DIR}/include"
  "${LDCP_SDK_DIR}/include"
)

add_executable(ltme_node src/ltme_node.cpp)

ament_target_dependencies(ltme_node
  std_srvs
  rclcpp
  ltme_interfaces
  sensor_msgs
  std_msgs
  rosidl_default_generators
  # ldcp_sdk
)
target_link_libraries(ltme_node ldcp_sdk)

#############
## Install ##
#############

install(TARGETS ltme_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
