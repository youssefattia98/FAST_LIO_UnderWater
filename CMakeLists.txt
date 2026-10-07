cmake_minimum_required(VERSION 3.8)
project(fast_lio)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Preserve the inherited optimization and flag precedence; CMake supplies C++17.
set(CMAKE_CXX_FLAGS "-O3 -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

set(UWFL2_MP_PROC_NUM "" CACHE STRING
  "OpenMP workers for scan correspondence; empty keeps the architecture default")
if("${UWFL2_MP_PROC_NUM}" STREQUAL "")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    if(N GREATER 4)
      set(UWFL2_EFFECTIVE_MP_PROC_NUM 3)
    elseif(N GREATER 3)
      set(UWFL2_EFFECTIVE_MP_PROC_NUM 2)
    else()
      set(UWFL2_EFFECTIVE_MP_PROC_NUM 1)
    endif()
  else()
    if(N GREATER 3)
      set(UWFL2_EFFECTIVE_MP_PROC_NUM 2)
    else()
      set(UWFL2_EFFECTIVE_MP_PROC_NUM 1)
    endif()
  endif()
else()
  if(NOT UWFL2_MP_PROC_NUM MATCHES "^[1-9][0-9]*$")
    message(FATAL_ERROR "UWFL2_MP_PROC_NUM must be a positive integer")
  endif()
  set(UWFL2_EFFECTIVE_MP_PROC_NUM ${UWFL2_MP_PROC_NUM})
endif()
if(UWFL2_EFFECTIVE_MP_PROC_NUM GREATER 1)
  add_definitions(-DMP_EN)
endif()
add_definitions(-DMP_PROC_NUM=${UWFL2_EFFECTIVE_MP_PROC_NUM})
message("OpenMP correspondence workers: ${UWFL2_EFFECTIVE_MP_PROC_NUM}")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  ament_index_cpp
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  pcl_conversions
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree filters)
find_package(GTSAM 4.2 REQUIRED MODULE)
find_package(yaml-cpp REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
  "srv/InjectLoop.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_library(uwfl2_ikdtree include/ikd-Tree/ikd_Tree.cpp)
target_include_directories(uwfl2_ikdtree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(uwfl2_ikdtree PUBLIC ${PCL_LIBRARIES})

add_library(uwfl2_loop_closure
  src/loop_closure/config_profile.cpp
  src/loop_closure/loop_closure_manager.cpp
  src/loop_closure/latest_scan_registration.cpp
  src/loop_closure/pose_graph.cpp
  src/loop_closure/shadow_map.cpp
  src/loop_closure/std_detector.cpp
)
target_include_directories(uwfl2_loop_closure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(uwfl2_loop_closure PUBLIC Eigen3::Eigen gtsam yaml-cpp uwfl2_ikdtree ${PCL_LIBRARIES})

add_executable(fastlio_mapping src/main.cpp src/laserMapping.cpp src/preprocess.cpp
  src/auxiliary_sensor_fusion.cpp src/IMU_Processing.cpp)
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} Eigen3::Eigen uwfl2_loop_closure)

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Interface type support for older ROS distros
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})

if(BUILD_TESTING)
  find_program(UWFL2_TEST_PYTHON python3 REQUIRED)
  add_test(NAME test_benchmark_tools
    COMMAND ${UWFL2_TEST_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_benchmark_tools.py)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_loop_closure_core test/test_loop_closure_core.cpp)
  target_include_directories(test_loop_closure_core PRIVATE include)
  target_link_libraries(test_loop_closure_core uwfl2_loop_closure)

  ament_add_gtest(test_std_detector test/test_std_detector.cpp)
  target_include_directories(test_std_detector PRIVATE include)
  target_link_libraries(test_std_detector uwfl2_loop_closure)

  ament_add_gtest(test_lidar_scan_quality test/test_lidar_scan_quality.cpp)
  target_include_directories(test_lidar_scan_quality PRIVATE include)

  ament_add_gtest(test_corrected_map_visualization
    test/test_corrected_map_visualization.cpp)
  target_include_directories(test_corrected_map_visualization PRIVATE include)

  ament_add_gtest(test_sensor_parameter_utils
    test/test_sensor_parameter_utils.cpp)
  target_include_directories(test_sensor_parameter_utils PRIVATE include)

  ament_add_gtest(test_odometry_covariance
    test/test_odometry_covariance.cpp)
  target_include_directories(test_odometry_covariance PRIVATE include)

  ament_add_gtest(test_estimator_characterization test/test_estimator_characterization.cpp
    src/auxiliary_sensor_fusion.cpp src/IMU_Processing.cpp)
  target_include_directories(test_estimator_characterization PRIVATE include src ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_estimator_characterization ${dependencies})
  if(COMMAND rosidl_get_typesupport_target)
    target_link_libraries(test_estimator_characterization ${cpp_typesupport_target})
  else()
    rosidl_target_interfaces(test_estimator_characterization ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()

  ament_add_gtest(test_preprocess_characterization
    test/test_preprocess_characterization.cpp src/preprocess.cpp)
  target_include_directories(test_preprocess_characterization PRIVATE src ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_preprocess_characterization ${PCL_LIBRARIES})
  ament_target_dependencies(test_preprocess_characterization rclcpp sensor_msgs pcl_conversions)

  ament_add_gtest(test_header_link
    test/test_header_link.cpp test/header_link_other.cpp src/auxiliary_sensor_fusion.cpp
    src/IMU_Processing.cpp)
  target_include_directories(test_header_link PRIVATE include src ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_header_link ${dependencies})
  if(COMMAND rosidl_get_typesupport_target)
    target_link_libraries(test_header_link ${cpp_typesupport_target})
  else()
    rosidl_target_interfaces(test_header_link ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
endif()

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping uwfl2_loop_closure uwfl2_ikdtree
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
