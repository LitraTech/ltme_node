cmake_minimum_required(VERSION 2.8.3)
project(ltme_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    message_generation
    std_msgs
)

add_service_files(
  DIRECTORY srv
  FILES
    QuerySerial.srv
    QueryFirmwareVersion.srv
    QueryHardwareVersion.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

###########
## Build ##
###########

set(LDCP_SDK_DIR "lib/ldcp_sdk")
add_subdirectory(${LDCP_SDK_DIR})

include_directories(
  "${PROJECT_SOURCE_DIR}/include"
  "${LDCP_SDK_DIR}/Sources/include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/ltme_node.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ldcp_sdk
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
