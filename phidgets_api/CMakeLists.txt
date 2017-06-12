cmake_minimum_required(VERSION 2.8.3)

project(phidgets_api)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(libphidget21 REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS}
  ${libphidget21_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${libphidget21_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${libphidget21_LIBRARIES})

add_library(phidgets_api src/encoder.cpp src/imu.cpp src/ir.cpp src/motor.cpp
  src/phidget.cpp)

target_link_libraries(phidgets_api ${LIBS})

install(TARGETS phidgets_api
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h")

install(DIRECTORY share/ DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(libphidget21)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(phidgets_api ${LIBS})

ament_package()
