cmake_minimum_required(VERSION 3.8)
project(grid_drive)

# Dijkstra / 원호 후보는 최적화 없이(-O0) 빌드하면 느리다
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(obstacle_detection REQUIRED)   # color_maps.hpp (색 방식 관측)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)   # third_party/vision_opencv (OpenCV 4.8)
find_package(interfaces REQUIRED)

# 주행 노드: 카메라 -> 색 + 도로 세그멘테이션 관측 -> grid_planner -> cmd_vel
add_executable(grid_drive_node src/grid_drive_node.cpp)
target_include_directories(grid_drive_node PRIVATE include)
target_link_libraries(grid_drive_node ${OpenCV_LIBS})
ament_target_dependencies(grid_drive_node rclcpp sensor_msgs geometry_msgs nav_msgs cv_bridge
  interfaces obstacle_detection)

# TensorRT (젯슨 JetPack) 가 있으면 도로 모델 추론. 없으면 (노트북 등) 색 방식만
find_package(CUDAToolkit QUIET)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
find_library(NVINFER_LIBRARY nvinfer)
set(GRID_DRIVE_EXTRA_TARGETS)
if(CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND NVINFER_LIBRARY)
  message(STATUS "grid_drive: TensorRT 도로 모델 사용 (${NVINFER_LIBRARY})")
  target_compile_definitions(grid_drive_node PRIVATE GRID_DRIVE_TENSORRT)
  target_include_directories(grid_drive_node SYSTEM PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(grid_drive_node ${NVINFER_LIBRARY} CUDA::cudart)
  # 도로 엔진 마스크를 jpg 폴더에 뽑는 도구 (ultralytics 결과와 비교)
  add_executable(road_seg_frames tools/road_seg_frames.cpp)
  target_include_directories(road_seg_frames PRIVATE include)
  target_include_directories(road_seg_frames SYSTEM PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(road_seg_frames ${OpenCV_LIBS} ${NVINFER_LIBRARY} CUDA::cudart)
  list(APPEND GRID_DRIVE_EXTRA_TARGETS road_seg_frames)
else()
  message(STATUS "grid_drive: TensorRT 없음 -> 색 방식만")
endif()

# 녹화 프레임 (jpg 폴더) 으로 계획기를 돌려 디버그 그림 / 시간을 남기는 도구 (ROS 없이)
add_executable(replay_frames tools/replay_frames.cpp)
target_include_directories(replay_frames PRIVATE include)
target_link_libraries(replay_frames ${OpenCV_LIBS})
ament_target_dependencies(replay_frames obstacle_detection)

install(TARGETS replay_frames grid_drive_node ${GRID_DRIVE_EXTRA_TARGETS}
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_planner test/test_grid_planner.cpp)
  target_include_directories(test_grid_planner PRIVATE include)
  target_link_libraries(test_grid_planner ${OpenCV_LIBS})
  ament_target_dependencies(test_grid_planner obstacle_detection)
  ament_add_gtest(test_road_seg test/test_road_seg.cpp)
  target_include_directories(test_road_seg PRIVATE include)
  target_link_libraries(test_road_seg ${OpenCV_LIBS})
endif()

ament_export_include_directories(include)
ament_package()
