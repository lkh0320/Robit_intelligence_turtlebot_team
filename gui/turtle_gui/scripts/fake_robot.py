#!/usr/bin/env python3
# 로봇 없이 GUI를 시험하기 위한 가짜 로봇 노드 (control_node + stm32_bridge_node + camera_node 흉내)
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
        super().__init__('fake_robot')
        self.declare_parameter('max_linear_mps', 0.2)
        self.declare_parameter('obstacle_stop_mm', 150)
        self.declare_parameter('hsv_low', [0, 0, 0])
        self.declare_parameter('line_kp', 0.005)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('mode_name', 'line')

        self.psd_pub = self.create_publisher(PsdArray, '/sensor/psd', qos_profile_sensor_data)
        self.motor_pub = self.create_publisher(MotorState, '/motor/state', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(RobotState, '/robot/state', 10)
        self.vision_pub = self.create_publisher(VisionResult, '/vision/result', 10)
        self.cam_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', qos_profile_sensor_data)
        self.debug_pub = self.create_publisher(CompressedImage, '/vision/debug_image/compressed', qos_profile_sensor_data)

        self.create_subscription(Twist, '/cmd_vel_manual', self.on_cmd, 10)
        self.create_service(SetBool, '/robot/run', self.on_run)
        self.create_service(SetBool, '/robot/estop', self.on_estop)
        self.create_service(SetMode, '/robot/set_mode', self.on_mode)

        self.mode = RobotState.MODE_MANUAL
        self.running = False
        self.estop = False
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = 0.0
        self.start_time = time.time()

        self.create_timer(0.05, self.publish_sensor)
        self.create_timer(0.2, self.publish_state)
        self.create_timer(1.0 / 15.0, self.publish_image)
        self.get_logger().info('fake_robot 시작 (cv2 %s)' % ('있음' if cv2 else '없음: 영상 생략'))

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_time = time.time()

    def on_run(self, request, response):
        self.running = request.data
        response.success = True
        response.message = 'running=%s' % self.running
        return response

    def on_estop(self, request, response):
        self.estop = request.data
        if self.estop:
            self.running = False  # 비상정지 시 실행도 정지
        response.success = True
        response.message = 'estop=%s' % self.estop
        return response

    def on_mode(self, request, response):
        if request.mode not in (RobotState.MODE_MANUAL, RobotState.MODE_AUTO):
            response.success = False
            response.message = 'unknown mode'
            return response
        self.mode = request.mode
        response.success = True
        response.message = 'ok'
        return response

    def wheel_speed(self):
        if not self.running or self.estop:
            return 0.0, 0.0
        if self.mode == RobotState.MODE_MANUAL:
            if time.time() - self.last_cmd_time > 0.5:
                return 0.0, 0.0  # 수동 명령 끊기면 정지
            v, w = self.cmd
        else:
            v, w = 0.1, 0.5 * math.sin(time.time() - self.start_time)
        half = 0.16 / 2.0
        return v - w * half, v + w * half

    def publish_sensor(self):
        t = time.time() - self.start_time
        psd = PsdArray()
        psd.header.stamp = self.get_clock().now().to_msg()
        psd.distance_mm = [int(400 + 300 * math.sin(t + k)) for k in (0.0, 2.0, 4.0)]
        psd.raw = [int(4095 * 80 / max(d, 80)) for d in psd.distance_mm]
        self.psd_pub.publish(psd)

        left, right = self.wheel_speed()
        motor = MotorState()
        motor.header.stamp = psd.header.stamp
        motor.left_mps = float(left)
        motor.right_mps = float(right)
        self.motor_pub.publish(motor)

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
