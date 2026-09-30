#!/usr/bin/env python3
"""로봇 없이 turtle_gui 를 시험하기 위한 가짜 로봇 노드.

발행: image_raw/compressed, vision/lane_debug/compressed, vision/object_debug/compressed,
      psd, dxl_state, 비전 인식 결과 6종
구독: cmd_vel -> dxl_state 속도에 반영 (GUI 송신 확인용)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import (Barrier, DxlState, LaneInfo, ParkingSpot, PsdArray, Sign,
                            StopLine, TrafficLight)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

WHEEL_SEPARATION = 0.160


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')
        qos = qos_profile_sensor_data
        self.image_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', qos)
        self.lane_img_pub = self.create_publisher(
            CompressedImage, 'vision/lane_debug/compressed', qos)
        self.object_img_pub = self.create_publisher(
            CompressedImage, 'vision/object_debug/compressed', qos)
        self.psd_pub = self.create_publisher(PsdArray, 'psd', qos)
        self.dxl_pub = self.create_publisher(DxlState, 'dxl_state', qos)
        self.lane_pub = self.create_publisher(LaneInfo, 'lane_info', qos)
        self.light_pub = self.create_publisher(TrafficLight, 'traffic_light', qos)
        self.stop_pub = self.create_publisher(StopLine, 'stop_line', qos)
        self.sign_pub = self.create_publisher(Sign, 'sign', qos)
        self.barrier_pub = self.create_publisher(Barrier, 'barrier', qos)
        self.parking_pub = self.create_publisher(ParkingSpot, 'parking_spot', qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)

        self.cmd = Twist()
        self.cmd_time = self.get_clock().now()
        self.t = 0.0
        self.create_timer(1.0 / 30.0, self.on_image)
        self.create_timer(1.0 / 20.0, self.on_sensors)
        if cv2 is None:
            self.get_logger().warn('OpenCV 없음: 카메라 영상은 발행하지 않습니다')
        self.get_logger().info('fake_robot 시작')

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()
        self.get_logger().info(f'cmd_vel 수신 v={msg.linear.x:.2f} w={msg.angular.z:.2f}')

    def stamp(self, msg, frame='base_link'):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        return msg

    def on_image(self):
        if cv2 is None:
            return
        self.t += 1.0 / 30.0
        img = np.full((240, 320, 3), 60, np.uint8)
        # 흰 차선 두 줄 + 움직이는 원
        shift = int(30 * math.sin(self.t))
        left = ((100 + shift, 240), (140 + shift, 100))
        right = ((220 + shift, 240), (180 + shift, 100))
        cv2.line(img, *left, (255, 255, 255), 6)
        cv2.line(img, *right, (255, 255, 255), 6)
        ball = (int(160 + 120 * math.cos(self.t)), 50)
        cv2.circle(img, ball, 15, (0, 0, 255), -1)
        cv2.putText(img, f'fake {self.t:6.1f}s', (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1)
        self.publish_jpeg(self.image_pub, img)

        # 선 검출 화면 흉내: 검출한 선 + 차선 중앙 진행 벡터
        lane = img.copy()
        cv2.line(lane, *left, (0, 255, 0), 2)
        cv2.line(lane, *right, (0, 255, 0), 2)
        bottom = ((left[0][0] + right[0][0]) // 2, 240)
        top = ((left[1][0] + right[1][0]) // 2, 100)
        cv2.arrowedLine(lane, bottom, top, (255, 0, 255), 3, tipLength=0.15)
        self.publish_jpeg(self.lane_img_pub, lane)

        # 객체 인식 화면 흉내: 공 주변 바운딩 박스 + 라벨
        obj = img.copy()
        cv2.rectangle(obj, (ball[0] - 20, ball[1] - 20), (ball[0] + 20, ball[1] + 20),
                      (0, 255, 255), 2)
        cv2.putText(obj, 'ball 0.92', (ball[0] - 20, ball[1] + 35), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (0, 255, 255), 1)
        self.publish_jpeg(self.object_img_pub, obj)

    def publish_jpeg(self, pub, img):
        ok, buf = cv2.imencode('.jpg', img)
        if ok:
            msg = self.stamp(CompressedImage(), 'camera')
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            pub.publish(msg)

    def on_sensors(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        phase = int(t / 3) % 3   # 3초마다 인식 결과 순환

        psd = self.stamp(PsdArray())
        psd.left = 0.4 + 0.3 * math.sin(t)
        psd.front = 0.4 + 0.3 * math.sin(t + 2.0)
        psd.right = 0.4 + 0.3 * math.sin(t + 4.0)
        self.psd_pub.publish(psd)

        # cmd_vel 이 0.5초 이상 끊기면 정지 (stm_bridge 와 동일)
        v = w = 0.0
        if (self.get_clock().now() - self.cmd_time).nanoseconds < 5e8:
            v, w = self.cmd.linear.x, self.cmd.angular.z
        dxl = self.stamp(DxlState())
        dxl.left_velocity = v - w * WHEEL_SEPARATION / 2.0
        dxl.right_velocity = v + w * WHEEL_SEPARATION / 2.0
        self.dxl_pub.publish(dxl)

        lane = self.stamp(LaneInfo(), 'camera')
        lane.detected = True
        lane.offset = 0.3 * math.sin(t)
        lane.angle = 0.2 * math.cos(t)
        lane.confidence = 0.9
        self.lane_pub.publish(lane)

        light = self.stamp(TrafficLight(), 'camera')
        light.state = [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN][phase]
        light.confidence = 0.8
        self.light_pub.publish(light)

        stop = self.stamp(StopLine(), 'camera')
        stop.detected = phase == 0
        stop.y_ratio = 0.7 if stop.detected else 0.0
        stop.confidence = 0.85 if stop.detected else 0.0
        self.stop_pub.publish(stop)

        sign = self.stamp(Sign(), 'camera')
        sign.type = [Sign.NONE, Sign.LEFT, Sign.RIGHT][phase]
        sign.area_ratio = 0.05 * phase
        sign.confidence = 0.7 if phase else 0.0
        self.sign_pub.publish(sign)

        barrier = self.stamp(Barrier(), 'camera')
        barrier.detected = phase != 2
        barrier.state = Barrier.CLOSED if phase == 0 else Barrier.OPEN
        barrier.confidence = 0.75
        self.barrier_pub.publish(barrier)

        park = self.stamp(ParkingSpot(), 'camera')
        park.detected = phase != 0
        park.empty_side = [ParkingSpot.NONE, ParkingSpot.LEFT, ParkingSpot.RIGHT][phase]
        park.offset = 0.1
        park.confidence = 0.6
        self.parking_pub.publish(park)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
