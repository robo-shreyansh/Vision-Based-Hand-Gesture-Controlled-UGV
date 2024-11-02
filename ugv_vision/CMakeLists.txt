cmake_minimum_required(VERSION 3.8)
project(ugv_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include_directories(include)
# ament_target_dependencies()
# 
# install(TARGETS
  # cpp_exec
  # DESTINATION lib/${PROJECT_NAME})
# 

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/pc_viewer_node.py
  scripts/ugv_cam_node.py
  DESTINATION lib/${PROJECT_NAME}/
)

 

ament_package()
