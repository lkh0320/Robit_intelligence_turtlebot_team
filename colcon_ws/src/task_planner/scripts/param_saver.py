#!/usr/bin/env python3
"""turtle_gui "YAML 저장" 을 받아 로봇(Jetson) 의 config yaml 에 값을 써 넣는 노드.

GUI 는 노트북에서 돌아 Jetson 파일을 직접 못 고치므로, 이 노드가 Jetson 에서 대신 쓴다.
  구독: param_saver/save   (std_msgs/String)  GUI "YAML 복사" 와 같은 형식
          grid_drive:
            ros__parameters:
              planner:
                linear: 0.3
  발행: param_saver/result (std_msgs/String)  JSON {"node", "ok", "names", "msg"}

어느 파일에 쓰나: 작업공간 src/**/config/*.yaml 중 맨 위 키가 노드 이름인 파일 (grid_drive.yaml 등).
  install 쪽 복사본이 링크가 아니면 (--symlink-install 이 아닌 패키지) 거기에도 같이 써서
  다시 빌드하지 않아도 다음 실행부터 반영된다.
쓰는 방법: 줄 단위로 그 값만 바꾼다 (주석 / 줄 맞춤 / 순서 유지). 파일에 없던 파라미터는 해당 묶음 끝에 추가.
src 파일은 git 이 있으니 되돌리려면 git diff / git checkout.
"""
import glob
import json
import os
import re

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml

KEY_LINE = re.compile(r'^(\s*)([A-Za-z0-9_./-]+):(.*)$')


def flatten(prefix, value, out):
    """{'planner': {'linear': 0.3}} -> {'planner.linear': 0.3}"""
    if isinstance(value, dict):
        for k, v in value.items():
            flatten(f'{prefix}.{k}' if prefix else str(k), v, out)
    else:
        out[prefix] = value


def yaml_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, float):
        s = repr(v)
        return s if any(c in s for c in '.einf') else s + '.0'
    if isinstance(v, int):
        return str(v)
    if isinstance(v, str):
        # 그대로 써도 문자열로 다시 읽히면 따옴표 없이 (주석 # / 앞뒤 공백 등이 있으면 따옴표)
        if v and '#' not in v and v.strip() == v and isinstance(yaml.safe_load(v), str):
            return v
        return json.dumps(v, ensure_ascii=False)
    return json.dumps(v, ensure_ascii=False)   # 배열 -> [1, 2]


def split_value(rest):
    """': 0.3      # 설명' 의 ':' 뒤 -> (앞 공백, 값, 값 뒤 공백 + 주석)"""
    m = re.match(r'^(\s*)(.*?)(\s+#.*)?$', rest)
    return m.group(1), m.group(2), m.group(3) or ''


def scan(lines):
    """줄마다 (경로 튜플) 을 구한다. 반환: {경로: 줄 번호}, {묶음 경로: (들여쓰기, 마지막 줄 번호)}"""
    leaves, blocks = {}, {}
    stack = []   # (들여쓰기, 키)
    for i, line in enumerate(lines):
        m = KEY_LINE.match(line)
        if not m or line.lstrip().startswith('#'):
            continue
        indent, key, rest = len(m.group(1)), m.group(2), m.group(3)
        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = tuple(k for _, k in stack) + (key,)
        for depth in range(1, len(path)):   # 이 줄을 품은 묶음들의 끝 줄 갱신
            ind, _ = blocks[path[:depth]]
            blocks[path[:depth]] = (ind, i)
        _, value, _ = split_value(rest)
        if value == '':
            blocks[path] = (indent, i)
            stack.append((indent, key))
        else:
            leaves[path] = i
    return leaves, blocks


def update_lines(lines, node, values):
    """lines 를 고쳐 반환. values = {'planner.linear': 0.3}"""
    for name, v in sorted(values.items()):
        leaves, blocks = scan(lines)
        path = (node, 'ros__parameters') + tuple(name.split('.'))
        text = yaml_value(v)
        if path in leaves:
            i = leaves[path]
            m = KEY_LINE.match(lines[i])
            pre, old, comment = split_value(m.group(3))
            if comment:   # 주석 위치를 되도록 그대로 (값이 길어지면 한 칸만 띄움)
                pad = max(1, len(old) + len(comment) - len(comment.lstrip()) - len(text))
                comment = ' ' * pad + comment.lstrip()
            lines[i] = f'{m.group(1)}{m.group(2)}:{pre or " "}{text}{comment}'
            continue
        # 없는 파라미터: 가장 깊이 있는 묶음 끝에 (모자란 묶음 줄도 만들어) 추가
        depth = len(path) - 1
        while depth > 0 and path[:depth] not in blocks:
            depth -= 1
        if depth == 0:   # 파일에 이 노드가 없음: 맨 끝에 노드 / ros__parameters 줄부터
            while lines and lines[-1] == '':
                lines.pop()
            lines += [f'{node}:', '  ros__parameters:', '']
            depth, indent, at = 2, 2, len(lines) - 1
        else:
            indent, last = blocks[path[:depth]]
            at = last + 1
        add = []
        for d in range(depth, len(path)):
            ind = ' ' * (indent + 2 * (d - depth + 1))
            add.append(f'{ind}{path[d]}:' + ('' if d < len(path) - 1 else f' {text}'))
        lines = lines[:at] + add + lines[at:]
    return lines


class ParamSaver(Node):

    def __init__(self):
        super().__init__('param_saver')
        # 작업공간: install/task_planner/share/task_planner -> 네 단계 위
        share = get_package_share_directory('task_planner')
        default_ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        self.ws = os.path.expanduser(self.declare_parameter('workspace', default_ws).value)
        self.pub = self.create_publisher(String, 'param_saver/result', 10)
        self.create_subscription(String, 'param_saver/save', self.on_save, 10)
        self.get_logger().info(f'param_saver 시작 (작업공간 {self.ws})')

    def find_files(self, node):
        """노드 yaml: src 파일 + (링크가 아닌) install 복사본"""
        src = None
        for path in sorted(glob.glob(os.path.join(self.ws, 'src', '**', 'config', '*.yaml'),
                                     recursive=True)):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f)
            except (OSError, yaml.YAMLError):
                continue
            if isinstance(data, dict) and node in data:
                src = path
                break
        if src is None:
            return []
        files = [src]
        pattern = os.path.join(self.ws, 'install', '*', 'share', '*', 'config', os.path.basename(src))
        for inst in glob.glob(pattern):
            if not os.path.islink(inst) and os.path.realpath(inst) != os.path.realpath(src):
                files.append(inst)
        return files

    def on_save(self, msg):
        node, names = '', []
        try:
            data = yaml.safe_load(msg.data)
            node = next(iter(data))
            values = {}
            flatten('', data[node]['ros__parameters'], values)
            names = sorted(values)
            files = self.find_files(node)
            if not files:
                raise RuntimeError(f'{node} 의 config yaml 을 {self.ws}/src 에서 못 찾음')
            for path in files:
                with open(path) as f:
                    lines = f.read().split('\n')
                lines = update_lines(lines, node, values)
                text = '\n'.join(lines)
                check = {}
                flatten('', yaml.safe_load(text)[node]['ros__parameters'], check)
                bad = [n for n in names if check.get(n) != values[n]]
                if bad:   # 쓰기 전에 다시 읽어 확인. 틀리면 파일은 그대로
                    raise RuntimeError(f'{path} 고친 결과가 이상함: {bad}')
                with open(path, 'w') as f:
                    f.write(text)
            rel = ', '.join(os.path.relpath(p, self.ws) for p in files)
            result = {'node': node, 'ok': True, 'names': names, 'msg': f'{len(names)}개 저장 -> {rel}'}
            self.get_logger().info(f'{node}: {result["msg"]} ({", ".join(names)})')
        except Exception as e:   # noqa: B902  어떤 실패든 GUI 에 알림
            result = {'node': node, 'ok': False, 'names': names, 'msg': str(e)}
            self.get_logger().error(f'저장 실패: {e}')
        self.pub.publish(String(data=json.dumps(result, ensure_ascii=False)))


def main():
    rclpy.init()
    node = ParamSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
