cmake_minimum_required(VERSION 3.8)
project(cross_traffic_manager_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# traffic_observer ノードを登録
add_executable(traffic_observer_cpp src/traffic_observer_cpp.cpp)
ament_target_dependencies(traffic_observer_cpp rclcpp std_msgs)

add_executable(traffic_judgementer_cpp src/traffic_judgementer_cpp.cpp)
ament_target_dependencies(traffic_judgementer_cpp rclcpp std_msgs std_srvs yaml-cpp)

# 必ずリンク対象としてyaml-cppを追加
target_link_libraries(traffic_judgementer_cpp yaml-cpp)

# 実行可能ファイルをインストール
install(TARGETS
  traffic_observer_cpp
  traffic_judgementer_cpp
  DESTINATION lib/${PROJECT_NAME}
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
