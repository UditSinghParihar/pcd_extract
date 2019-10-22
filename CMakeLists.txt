cmake_minimum_required(VERSION 2.8.3)
project(pcd_extract)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp sensor_msgs nav_msgs tf geometry_msgs image_transport cv_bridge image_geometry)
find_package(Boost REQUIRED COMPONENTS signals)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

add_executable(pcd_extract src/pcd_extract.cpp)
add_executable(pcd_sync src/pcd_sync.cpp)
add_executable(carto_cloud src/carto_cloud.cpp)
add_executable(tf_sub src/tf_sub.cpp)
add_executable(tf_label src/tf_label.cpp)
add_executable(cloud_from_depth src/cloud_from_depth.cpp)
add_executable(sync_images src/sync_images.cpp)
add_executable(tf_label_rgb src/tf_label_rgb.cpp)

target_link_libraries(pcd_extract ${catkin_LIBRARIES} )
target_link_libraries(pcd_sync ${catkin_LIBRARIES} )
target_link_libraries(carto_cloud ${catkin_LIBRARIES})
target_link_libraries(tf_sub ${catkin_LIBRARIES})
target_link_libraries(tf_label ${catkin_LIBRARIES})
target_link_libraries(cloud_from_depth ${catkin_LIBRARIES})
target_link_libraries(sync_images ${catkin_LIBRARIES})
target_link_libraries(tf_label_rgb ${catkin_LIBRARIES})