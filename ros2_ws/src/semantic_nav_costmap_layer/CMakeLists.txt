cmake_minimum_required(VERSION 3.8)
project(semantic_nav_costmap_layer)
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/semantic_layer.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} nav2_costmap_2d nav2_msgs nav2_util
  pluginlib rclcpp tf2 tf2_ros tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d semantic_layer.xml)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(nav2_costmap_2d nav2_msgs nav2_util rclcpp tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_semantic_layer test/test_semantic_layer.cpp)
  target_link_libraries(test_semantic_layer ${PROJECT_NAME})
  ament_target_dependencies(test_semantic_layer pluginlib tf2_geometry_msgs)
endif()
ament_package()
