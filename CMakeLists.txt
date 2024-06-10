cmake_minimum_required(VERSION 3.14)
project(obstacle_dstar)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/obstacle_dstar/obstacle_dstar.hpp
  include/obstacle_dstar/visibility_control.hpp
  src/obstacle_dstar.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/obstacle_dstar/obstacle_dstar_node.hpp
  #include/obstacle_dstar/obstacle_dstar_node_copy.hpp
  src/obstacle_dstar_node.cpp
  #src/obstacle_dstar_node_copy.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "obstacle_dstar::ObstacleDstarNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_obstacle_dstar
    test/test_obstacle_dstar.cpp
  )
  ament_target_dependencies(test_obstacle_dstar ${TEST_SOURCES})
  target_link_libraries(test_obstacle_dstar ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
