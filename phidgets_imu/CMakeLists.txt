cmake_minimum_required(VERSION 2.8.3)
project(phidgets_imu)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(phidgets_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ros2_time REQUIRED)

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS} ${diagnostic_updater_INCLUDE_DIRS}
  ${phidgets_api_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${std_srvs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS} ${ros2_time_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${diagnostic_msgs_LIBRARY_DIRS}
  ${diagnostic_updater_LIBRARY_DIRS}
  ${phidgets_api_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS} ${sensor_msgs_LIBRARY_DIRS}
  ${std_msgs_LIBRARY_DIRS} ${std_srvs_LIBRARY_DIRS} ${tf2_LIBRARY_DIRS}
  ${ros2_time_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${diagnostic_msgs_LIBRARIES}
  ${diagnostic_updater_LIBRARIES} ${phidgets_api_LIBRARIES}
  ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES} ${tf2_LIBRARIES} ${ros2_time_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS thread)

add_library(phidgets_imu src/imu_ros_i.cpp)
add_library(phidgets_imu_nodelet src/phidgets_imu_nodelet.cpp)

add_executable(phidgets_imu_node src/phidgets_imu_node.cpp)

target_link_libraries(phidgets_imu ${LIBS} ${Boost_LIBRARIES})
target_link_libraries(phidgets_imu_nodelet ${LIBS} phidgets_imu)
target_link_libraries(phidgets_imu_node ${LIBS} phidgets_imu)

install(TARGETS phidgets_imu phidgets_imu_nodelet phidgets_imu_node
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

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

install(FILES phidgets_imu_nodelet.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(diagnostic_updater)
ament_export_dependencies(phidgets_api)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(ros2_time)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(phidgets_imu phidgets_imu_nodelet ${LIBS})

ament_package()
