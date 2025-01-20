#  minimum required cmake version: 3.1.15 support vs2019

cmake_minimum_required(VERSION 3.1.15)
project(myMultiDeviceSync)

add_executable(${PROJECT_NAME}
  src/MultiDeviceSync.cpp
  src/multiDevice_funcs.cpp
  src/cJSON.c
)

find_package(OpenCV REQUIRED)

# Set Orbbec SDK paths
set(OrbbecSDK_DIR /home/enes/Desktop/ORBBEC/OrbbecSDK)
set(OrbbecSDK_LIB_DIR ${OrbbecSDK_DIR}/lib/linux_x64)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_DIR}/include)
set(OrbbecSDK_INCLUDE_HPP_DIR ${OrbbecSDK_DIR}/include/libobsensor/hpp
                               ${OrbbecSDK_DIR}/examples/cpp)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${OrbbecSDK_INCLUDE_DIR}
  ${OrbbecSDK_INCLUDE_HPP_DIR}
)


# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${OrbbecSDK_LIB_DIR}/libOrbbecSDK.so
  ${OrbbecSDK_LIB_DIR}/libdepthengine.so
  ${OrbbecSDK_LIB_DIR}/libob_usb.so
  ${OrbbecSDK_LIB_DIR}/liblive555.so
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
