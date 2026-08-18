cmake_minimum_required(VERSION 3.8)
project(co_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 새 채점기를 만들면 이 목록에 파일 한 줄만 추가하면 됩니다.
# (각 파일이 CO_DRIVER_REGISTER_SCORER 로 스스로 등록합니다)
set(scorer_sources
  src/scorers/external_score.cpp       # external score topics (sync skeleton)
  src/scorers/localization_state.cpp   # Lost/Converging/Tracking state gate
  src/scorers/recovery_gate.cpp        # return only after sustained full health
  src/scorers/example_async.cpp        # async skeleton for heavy computations
)

add_executable(co_driver_node
  src/co_driver_node.cpp   # 메인 노드 — ROS 배선
  src/config.cpp           # 설정 — yaml + 토픽 목록 JSON
  src/compute.cpp          # 연산 — 응답곡선 / 점수 / 선택 / 후처리 / 맵·궤적 유틸
  ${scorer_sources}        # 채점기 — 각자 자기 토픽을 구독
)
target_include_directories(co_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(co_driver_node nlohmann_json::nlohmann_json)
ament_target_dependencies(co_driver_node ${dependencies})

install(TARGETS co_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Python helpers.
#   example_python_scorer   external scorer example - watches /drive, publishes scores
#   drive_monitor           arbitration monitor - which topic drives, and is it flapping
#   fake_*                  bench fixtures, for testing without a car
install(PROGRAMS
  scripts/example_python_scorer.py
  scripts/drive_monitor.py
  scripts/fake_localization_confidence.py
  scripts/fake_drive.py
  scripts/scan_qos_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
