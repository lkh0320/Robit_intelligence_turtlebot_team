# ros2/

Jetson에서 실행되는 ROS 2 패키지와, 노트북 GUI도 같이 쓰는 msg/srv 패키지를 둡니다.
빌드는 레포 루트에서 `bash scripts/build_robot.sh` 로 합니다.

## 패키지

```text
ros2/
├── turtle_interfaces/     msg/srv 정의 (Jetson, 노트북 둘 다 빌드)
├── turtle_bringup/        launch/robot.launch.py, config/ 설치
├── turtle_vision/         camera_node, vision_node
├── turtle_control/        control_node (모드, 주행 판단, 비상정지)
└── turtle_stm32_bridge/   stm32_bridge_node (시리얼 ↔ ROS 2)
```

노드끼리의 연결:

```text
camera_node ──/camera/image_raw──▶ vision_node ──/vision/result──▶ control_node ──/motor/cmd──▶ stm32_bridge_node ⇄ STM32
                                                                     ▲                               │
                                                     /sensor/psd ────┴───────────────────────────────┘
```

토픽/서비스/파라미터 전체 목록: [`../docs/ros2_interfaces.md`](../docs/ros2_interfaces.md)

## 패키지 생성 (처음 1회)

```bash
cd ros2
ros2 pkg create turtle_interfaces   --build-type ament_cmake
ros2 pkg create turtle_bringup      --build-type ament_cmake
ros2 pkg create turtle_vision       --build-type ament_cmake --dependencies rclcpp sensor_msgs cv_bridge image_transport turtle_interfaces
ros2 pkg create turtle_control      --build-type ament_cmake --dependencies rclcpp geometry_msgs std_srvs turtle_interfaces
ros2 pkg create turtle_stm32_bridge --build-type ament_cmake --dependencies rclcpp turtle_interfaces
```

## 규칙

### 1. OpenCV 로직은 ROS를 모르게 (turtle_vision)

```text
turtle_vision/
├── include/turtle_vision/core/   ← rclcpp include 금지. cv::Mat 넣으면 결과가 나오는 순수 함수/클래스
│   ├── preprocess.hpp
│   ├── line_detector.hpp
│   └── color_detector.hpp
├── src/core/                     ← 위 구현 (기능 추가 = 파일 추가)
├── src/camera_node.cpp           ← 웹캠 → /camera/image_raw
├── src/vision_node.cpp           ← 메시지 ↔ cv::Mat 변환, 파라미터 전달, core 호출만
└── test/                         ← 저장된 이미지로 core만 테스트 (로봇 없이 가능)
```

이렇게 하면 영상처리 코드를 ROS 없이 테스트할 수 있고, 나중에 알고리즘을 바꿔도 노드 코드는 거의 그대로입니다.

### 2. 통신 규약은 protocol/ 헤더를 include (turtle_stm32_bridge)

```cmake
# turtle_stm32_bridge/CMakeLists.txt
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../protocol)
```

패킷 구조체를 bridge 안에 다시 정의하지 마세요.

### 3. config/ 는 bringup이 설치

```cmake
# turtle_bringup/CMakeLists.txt
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/ DESTINATION share/${PROJECT_NAME}/config)
```

launch에서는 `get_package_share_directory('turtle_bringup') + '/config/vision.yaml'` 처럼 불러옵니다.

### 4. 파라미터는 선언해서 쓰기

- 모든 튜닝 값은 `declare_parameter`로 선언하고 `config/*.yaml`에 기본값을 둡니다.
- GUI에서 바꿀 수 있도록 `add_on_set_parameters_callback`으로 변경을 반영합니다.
- 포트, 해상도처럼 실행 중 바꾸면 안 되는 값은 `read_only = true`로 선언합니다.
