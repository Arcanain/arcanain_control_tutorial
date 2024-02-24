cmake_minimum_required(VERSION 3.8)
project(arcanain_control_tutorial)

# Set the RPATH
set(CMAKE_INSTALL_RPATH "/usr/local/lib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(CasADi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

# 01_casadi_optimization_example
add_executable(casadi_optimization_example src/01_casadi_optimization_example.cpp)
ament_target_dependencies(casadi_optimization_example rclcpp)
target_include_directories(casadi_optimization_example PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(casadi_optimization_example
  ${CASADI_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# 02_car_race_node
add_executable(car_race_node src/02_car_race_node.cpp)
ament_target_dependencies(car_race_node rclcpp)
target_include_directories(car_race_node PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(car_race_node
  ${CASADI_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# 03_cartpole_mpc_control
add_executable(cartpole_mpc_control src/03_cartpole_mpc_control.cpp)
ament_target_dependencies(cartpole_mpc_control rclcpp)
target_include_directories(cartpole_mpc_control PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(cartpole_mpc_control
  ${CASADI_LIBRARIES}
  ${PYTHON_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# 04_lateral_car_mpc_control
add_executable(lateral_car_mpc_control src/04_lateral_car_mpc_control.cpp)
ament_target_dependencies(lateral_car_mpc_control rclcpp)
target_include_directories(lateral_car_mpc_control PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(lateral_car_mpc_control
  ${CASADI_LIBRARIES}
  ${PYTHON_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# 05_single_obstacle_avoidance_car_mpc_control
add_executable(single_obstacle_avoidance_car_mpc_control src/05_single_obstacle_avoidance_car_mpc_control.cpp)
ament_target_dependencies(single_obstacle_avoidance_car_mpc_control rclcpp tf2_geometry_msgs tf2_ros nav_msgs geometry_msgs)
target_include_directories(single_obstacle_avoidance_car_mpc_control PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(single_obstacle_avoidance_car_mpc_control
  ${CASADI_LIBRARIES}
  ${PYTHON_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# 06_multiple_obstacle_avoidance_car_mpc_control
add_executable(multiple_obstacle_avoidance_car_mpc_control src/06_multiple_obstacle_avoidance_car_mpc_control.cpp)
ament_target_dependencies(multiple_obstacle_avoidance_car_mpc_control rclcpp tf2_geometry_msgs tf2_ros nav_msgs geometry_msgs)
target_include_directories(multiple_obstacle_avoidance_car_mpc_control PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(multiple_obstacle_avoidance_car_mpc_control
  ${CASADI_LIBRARIES}
  ${PYTHON_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# Install Cpp executables
install(TARGETS
  casadi_optimization_example
  car_race_node
  cartpole_mpc_control
  lateral_car_mpc_control
  single_obstacle_avoidance_car_mpc_control
  multiple_obstacle_avoidance_car_mpc_control
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

# Install other files
install(DIRECTORY
  launch
  rviz
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
