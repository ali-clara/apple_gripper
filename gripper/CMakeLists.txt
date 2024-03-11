cmake_minimum_required(VERSION 3.8)
project(gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# include cpp "include" directory
include_directories(include)

# create cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)

# install cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# install python modules
ament_python_install_package(${PROJECT_NAME})

# install python executables
install(PROGRAMS
  gripper/user.py
  gripper/suction_gripper.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# install config files
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

# install urdf files
install(DIRECTORY
    urdf
    DESTINATION share/${PROJECT_NAME}
)

# install mesh files
install(DIRECTORY
    meshes
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
