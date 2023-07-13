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

find_package(OpenSSL)

add_service_files(
  DIRECTORY srv
  FILES
    QueryModel.srv
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

add_executable(ltme_node
  "src/ltme_node.cpp"
)

add_dependencies(ltme_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(ltme_node
  ${catkin_LIBRARIES}
  ldcp_sdk
)

if(OPENSSL_FOUND)
  add_executable(update_firmware
    "src/update_firmware.cpp"
  )

  target_link_libraries(update_firmware
    ldcp_sdk
    OpenSSL::Crypto
  )
endif()

#############
## Install ##
#############

install(TARGETS ltme_node update_firmware
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
