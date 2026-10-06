cmake_minimum_required(VERSION 3.8)
project(sign_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)   # third_party/vision_opencv 에서 OpenCV 4.8 로 빌드한 것
find_package(interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

add_executable(sign_detection_node src/sign_detection_node.cpp)
target_include_directories(sign_detection_node PRIVATE include)
target_link_libraries(sign_detection_node ${OpenCV_LIBS})
ament_target_dependencies(sign_detection_node rclcpp sensor_msgs cv_bridge interfaces)

install(TARGETS sign_detection_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolo_classes test/test_yolo_classes.cpp)
  target_include_directories(test_yolo_classes PRIVATE include)
  ament_target_dependencies(test_yolo_classes interfaces)
  # 팀 공용 클래스 파일과 비교 (colcon_ws/src/vision/sign_detection -> 저장소 루트/yolo)
  target_compile_definitions(test_yolo_classes PRIVATE
    YOLO_CLASSES_TXT="${CMAKE_CURRENT_SOURCE_DIR}/../../../../yolo/classes.txt")

  ament_add_gtest(test_yolo_detector test/test_yolo_detector.cpp)
  target_include_directories(test_yolo_detector PRIVATE include)
  target_link_libraries(test_yolo_detector ${OpenCV_LIBS})
  ament_target_dependencies(test_yolo_detector interfaces)
endif()

ament_package()
