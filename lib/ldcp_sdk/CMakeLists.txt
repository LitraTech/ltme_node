cmake_minimum_required(VERSION 2.8.3)

project(ldcp_sdk)

option(BUILD_DEVICE_MANAGER "Build device manager" OFF)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-std=c++11)
  add_compile_options(-fvisibility=hidden)
endif()

set(SDK_SRC_DIR "src")
set(SDK_SRC
  "${SDK_SRC_DIR}/device_base.cpp"
  "${SDK_SRC_DIR}/device.cpp"
  "${SDK_SRC_DIR}/bootloader.cpp"
  "${SDK_SRC_DIR}/device_info.cpp"
  "${SDK_SRC_DIR}/location.cpp"
  "${SDK_SRC_DIR}/session.cpp"
  "${SDK_SRC_DIR}/transport.cpp"
)
if(BUILD_DEVICE_MANAGER)
  set(SDK_SRC ${SDK_SRC}
    "${SDK_SRC_DIR}/device_manager.cpp"
    "${SDK_SRC_DIR}/device_notifier.cpp"
  )
endif()
add_library(${PROJECT_NAME} STATIC ${SDK_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_include_directories(${PROJECT_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Asio/asio-1.18.0/include"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RapidJSON/rapidjson-1.1.0/include"
)
add_definitions(-DASIO_DISABLE_VISIBILITY)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(SDK_LIB_DEPS pthread)
  if(BUILD_DEVICE_MANAGER)
    set(SDK_LIB_DEPS ${SDK_LIB_DEPS} avahi-common avahi-client)
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  add_definitions(-D_WIN32_WINNT=0x0501 -DNOMINMAX)
  set(SDK_LIB_DEPS WS2_32.Lib)
  if(BUILD_DEVICE_MANAGER)
    set(BONJOUR_SDK_DIR "C:/Program Files/Bonjour SDK" CACHE STRING "Directory of Bonjour SDK")
    target_include_directories(${PROJECT_NAME} PRIVATE "${BONJOUR_SDK_DIR}/Include")
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
      set(SDK_LIB_DEPS ${SDK_LIB_DEPS} "${BONJOUR_SDK_DIR}/Lib/x64/dnssd.lib")
    else()
      set(SDK_LIB_DEPS ${SDK_LIB_DEPS} "${BONJOUR_SDK_DIR}/Lib/Win32/dnssd.lib")
    endif()
  endif()
endif()

target_link_libraries(${PROJECT_NAME} ${SDK_LIB_DEPS})
