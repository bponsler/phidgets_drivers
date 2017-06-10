cmake_minimum_required(VERSION 2.8.3)
project(libphidget21)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

file(MAKE_DIRECTORY include/${PROJECT_NAME})

include(ExternalProject)
externalproject_add(EP_${PROJECT_NAME}
  URL
  https://www.phidgets.com/downloads/libraries/libphidget_2.1.8.20151217.tar.gz
  URL_MD5 818ab2ff1de92ed9568a206e0e89657f
  CONFIGURE_COMMAND "./configure"
  SOURCE_DIR ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src
  BUILD_IN_SOURCE 1
  BUILD_COMMAND make
  "CFLAGS=-g -O2 -Wno-incompatible-pointer-types -Wno-deprecated-declarations"  # copy headers to devel space (catkin does not like headers in source space)
  
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/phidget21.h
  ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/phidget21.h  # copy libs, set-up soname chain
  
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-src/.libs/libphidget21.so.0.0.0
  ${CMAKE_INSTALL_PREFIX}/lib/libphidget21.so.0.0.0
  COMMAND ${CMAKE_COMMAND} -E create_symlink libphidget21.so.0
  ${CMAKE_INSTALL_PREFIX}/lib/libphidget21.so
  COMMAND ${CMAKE_COMMAND} -E create_symlink libphidget21.so.0.0.0
  ${CMAKE_INSTALL_PREFIX}/lib/libphidget21.so.0
  INSTALL_COMMAND "")

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(PROGRAMS
  ${CMAKE_INSTALL_PREFIX}/lib/libphidget21.so
  ${CMAKE_INSTALL_PREFIX}/lib/libphidget21.so.0
  ${CMAKE_INSTALL_PREFIX}/lib/libphidget21.so.0.0.0
  DESTINATION lib)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS} ${CMAKE_INSTALL_PREFIX}/include)
ament_export_libraries(phidget21)

ament_package()
