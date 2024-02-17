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
find_package(CasADi REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)

# cartpole_mpc
add_executable(cartpole_mpc src/cartpole_mpc.cpp)
ament_target_dependencies(cartpole_mpc rclcpp)
target_include_directories(cartpole_mpc PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(cartpole_mpc
  ${CASADI_LIBRARIES}
  ${PYTHON_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# car_race_node
add_executable(car_race_node src/car_race_node.cpp)
ament_target_dependencies(car_race_node rclcpp)
target_include_directories(car_race_node PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(car_race_node
  ${CASADI_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# casadi_opti_example
add_executable(casadi_opti_example src/casadi_opti_example.cpp)
ament_target_dependencies(casadi_opti_example rclcpp)
target_include_directories(casadi_opti_example PRIVATE
  ${CASADI_INCLUDE_DIR}
)
target_link_libraries(casadi_opti_example
  ${CASADI_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

# Install Cpp executables
install(TARGETS
  cartpole_mpc
  car_race_node
  casadi_opti_example
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
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
