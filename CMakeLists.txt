cmake_minimum_required(VERSION 3.5)
project(simple_pointcloud_mapper)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(map_node src/map_node.cpp)

target_link_libraries(map_node
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)

target_include_directories(map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_options(map_node PRIVATE "${OpenMP_CXX_FLAGS}")

ament_target_dependencies(map_node 
  Eigen3 
  rclcpp 
  std_msgs
  std_srvs
  geometry_msgs 
  nav_msgs
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
)


install(TARGETS
  map_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
