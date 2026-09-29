#!/usr/bin/env python3
# 로봇 없이 GUI를 시험하기 위한 가짜 로봇 노드 (control_node + stm32_bridge_node + camera_node 흉내)
#
# 실행: ros2 run turtle_gui fake_robot.py   (다른 터미널에서 ros2 run turtle_gui turtle_gui)
#
# 하는 일
#  - 발행: /sensor/psd, /motor/state (20 Hz), /robot/state, /vision/result (5 Hz), 카메라 영상 2종 (15 Hz)
#  - 서비스: /robot/run, /robot/estop, /robot/set_mode 에 진짜 control_node 처럼 응답
#  - 구독: /cmd_vel_manual → 수동 모드에서 받은 명령을 바퀴 속도로 바꿔 /motor/state 에 반영
#  - 파라미터: GUI 파라미터 탭 테스트용 (값을 바꿔도 동작에는 영향 없음)
#
# 실제 노드를 만들 때 이 파일을 "토픽/서비스를 어떻게 발행·응답하는지" 예제로 참고해도 됨
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import SetBool
from turtle_interfaces.msg import MotorState, PsdArray, RobotState, VisionResult
from turtle_interfaces.srv import SetMode

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None  # OpenCV 없으면 영상만 생략


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')  # 노드 이름 → GUI 파라미터 탭에서 /fake_robot 으로 선택

        # GUI 파라미터 탭 테스트용 파라미터 (타입별로 하나씩: double, int, int 배열, bool, string)
        self.declare_parameter('max_linear_mps', 0.2)
        self.declare_parameter('obstacle_stop_mm', 150)
        self.declare_parameter('hsv_low', [0, 0, 0])
        self.declare_parameter('line_kp', 0.005)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('mode_name', 'line')

        # 발행자. 센서/영상처럼 자주 오고 하나쯤 빠져도 되는 데이터는 sensor_data QoS(best effort)
        self.psd_pub = self.create_publisher(PsdArray, '/sensor/psd', qos_profile_sensor_data)
        self.motor_pub = self.create_publisher(MotorState, '/motor/state', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(RobotState, '/robot/state', 10)
        self.vision_pub = self.create_publisher(VisionResult, '/vision/result', 10)
        self.cam_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(CompressedImage, '/vision/debug_image/compressed', qos_profile_sensor_data)

        # GUI → 로봇 방향 (구독, 서비스 서버)
        self.create_subscription(Twist, '/cmd_vel_manual', self.on_cmd, 10)
        self.create_service(SetBool, '/robot/run', self.on_run)
        self.create_service(SetBool, '/robot/estop', self.on_estop)
        self.create_service(SetMode, '/robot/set_mode', self.on_mode)

        # 가짜 로봇의 내부 상태
        self.mode = RobotState.MODE_MANUAL
        self.running = False
        self.estop = False
        self.cmd = (0.0, 0.0)  # 마지막 수동 명령 (linear m/s, angular rad/s)
        self.last_cmd_time = 0.0
        self.start_time = time.time()

        # 주기적으로 발행 (초 단위 주기, 콜백)
        self.create_timer(0.05, self.publish_sensor)
        self.create_timer(0.2, self.publish_state)
        self.create_timer(1.0 / 15.0, self.publish_image)
        self.get_logger().info('fake_robot 시작 (cv2 %s)' % ('있음' if cv2 else '없음: 영상 생략'))

    # /cmd_vel_manual 수신: 명령과 받은 시각 저장
    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = time.time()

    # /robot/run 서비스: request.data 가 true 면 실행, false 면 정지
    # 서비스 콜백은 response 를 채워서 return 해야 GUI 가 결과를 받음
    def on_run(self, request, response):
        self.running = request.data
        response.success = True
        response.message = 'running=%s' % self.running
        return response

    # /robot/estop 서비스: true 면 비상정지, false 면 해제
    def on_estop(self, request, response):
        self.estop = request.data
        if self.estop:
            self.running = False  # 비상정지 시 실행도 정지
        response.success = True
        response.message = 'estop=%s' % self.estop
        return response

    # /robot/set_mode 서비스: 정해진 모드(0, 1)가 아니면 거부
    def on_mode(self, request, response):
        if request.mode not in (RobotState.MODE_MANUAL, RobotState.MODE_AUTO):
            response.success = False
            response.message = 'unknown mode'
            return response
        self.mode = request.mode
        response.success = True
        response.message = 'ok'
        return response

    # 현재 상태로 좌/우 바퀴 속도 계산
    #  정지/비상정지 → 0, 수동 → GUI 명령, 자율 → 사인파로 흔들며 전진
    def wheel_speed(self):
        if not self.running or self.estop:
            return 0.0, 0.0
        if self.mode == RobotState.MODE_MANUAL:
            if time.time() - self.last_cmd_time > 0.5:
                return 0.0, 0.0  # 수동 명령 끊기면 정지
            v, w = self.cmd
        else:
            v, w = 0.1, 0.5 * math.sin(time.time() - self.start_time)
        # 차동 구동 공식: 바퀴 속도 = v ∓ w × (바퀴 간격 / 2)  (0.16 m = robot.yaml 의 wheel_base_m)
        half = 0.16 / 2.0
        return v - w * half, v + w * half

    # 20 Hz: 가짜 PSD 거리(사인파)와 바퀴 속도 발행
    def publish_sensor(self):
        t = time.time() - self.start_time
        psd = PsdArray()
        psd.header.stamp = self.get_clock().now().to_msg()
        psd.distance_mm = [int(400 + 300 * math.sin(t + k)) for k in (0.0, 2.0, 4.0)]
        psd.raw = [int(4095 * 80 / max(d, 80)) for d in psd.distance_mm]  # 가까울수록 raw 가 커지는 PSD 특성 흉내
        self.psd_pub.publish(psd)

        left, right = self.wheel_speed()
        motor = MotorState()
        motor.header.stamp = psd.header.stamp
        motor.left_mps = float(left)
        motor.right_mps = float(right)
        self.motor_pub.publish(motor)

    # 5 Hz: 로봇 상태와 가짜 비전 결과 발행
    def publish_state(self):
        t = time.time() - self.start_time
        state = RobotState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.mode = self.mode
        state.running = self.running
        state.estop = self.estop
        state.stm32_connected = True
        state.message = 'fake robot'
        self.state_pub.publish(state)

        vision = VisionResult()
        vision.header.stamp = state.header.stamp
        vision.detected = True
        vision.offset = float(math.sin(t * 0.7))
        vision.label = 'line'
        self.vision_pub.publish(vision)

    # 15 Hz: 가짜 카메라 영상(원본)과 검출 표시를 그린 영상(debug) 발행
    def publish_image(self):
        if cv2 is None:
            return
        t = time.time() - self.start_time
        img = np.full((480, 640, 3), 60, np.uint8)
        x = int(320 + 200 * math.sin(t * 0.7))
        cv2.line(img, (x, 480), (320, 200), (20, 20, 20), 40)  # 가짜 라인
        cv2.putText(img, 'FAKE CAMERA %.1fs' % t, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        self.cam_pub.publish(self.to_msg(img))

        debug = img.copy()
        cv2.circle(debug, (x, 440), 15, (0, 0, 255), -1)  # 검출 위치 표시
        cv2.line(debug, (320, 0), (320, 480), (0, 255, 0), 1)
        self.debug_pub.publish(self.to_msg(debug))

    # OpenCV 이미지 → JPEG 로 압축한 CompressedImage 메시지 (GUI 는 압축 영상만 받음)
    def to_msg(self, img):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
        return msg


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
