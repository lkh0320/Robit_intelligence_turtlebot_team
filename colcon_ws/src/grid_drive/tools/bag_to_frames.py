# 녹화 (rosbag) 에서 사진과 속도 명령을 꺼내 폴더에 저장한다
#   사용: python3 bag_to_frames.py <bag 폴더> <출력 폴더> [원본 몇 장마다 1장]
#   출력: raw/     원본 카메라 사진 (replay_frames 에 넣는 것)
#         debug/   grid_drive 디버그 화면 (계획한 경로가 그려진 것)
#         cmd.csv  시간, cmd_vel 직진 / 회전
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def open_bag(path):
    # bag 폴더 열기 (형식은 metadata 에서 알아서 찾음)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def image_to_bgr(msg):
    # sensor_msgs/Image -> OpenCV BGR 사진
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


def save_jpg(path, data):
    # 이미 jpg 인 바이트를 그대로 저장
    with open(path, 'wb') as f:
        f.write(bytes(data))


def main():
    if len(sys.argv) < 3:
        print('사용: python3 bag_to_frames.py <bag 폴더> <출력 폴더> [원본 몇 장마다 1장]')
        return
    bag, out = sys.argv[1], sys.argv[2]
    every = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    os.makedirs(out + '/raw', exist_ok=True)
    os.makedirs(out + '/debug', exist_ok=True)

    reader, types = open_bag(bag)
    t0 = None
    n_raw = n_dbg = n_cmd = 0
    k = 0
    with open(out + '/cmd.csv', 'w') as csv:
        csv.write('t,v,w\n')
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic not in types:
                continue
            if t0 is None:
                t0 = t
            sec = (t - t0) / 1e9
            name = f'{sec:08.3f}'     # 파일 이름 = 녹화 시작부터 초
            msg = deserialize_message(data, get_message(types[topic]))
            if topic.endswith('image_raw'):
                k += 1
                if k % every == 0:
                    cv2.imwrite(f'{out}/raw/{name}.jpg', image_to_bgr(msg), [cv2.IMWRITE_JPEG_QUALITY, 90])
                    n_raw += 1
            elif topic.endswith('debug/compressed'):
                save_jpg(f'{out}/debug/{name}.jpg', msg.data)
                n_dbg += 1
            elif topic.endswith('cmd_vel'):
                csv.write(f'{sec:.3f},{msg.linear.x:.4f},{msg.angular.z:.4f}\n')
                n_cmd += 1
    print(f'원본 {n_raw}장, 디버그 {n_dbg}장, 명령 {n_cmd}개 -> {out}')


if __name__ == '__main__':
    main()