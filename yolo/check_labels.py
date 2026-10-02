#!/usr/bin/env python3
"""
라벨 검사: 팀원이 만든 YOLO 라벨이 팀 공용 클래스 규칙에 맞는지 확인한다.

  1. classes.txt 와 data.yaml 의 클래스 순서가 같은지
  2. 라벨 파일(.txt)의 각 줄이 "번호 cx cy w h" 형식이고, 번호가 0~8, 좌표가 0~1 인지
  3. 라벨이 없는 이미지 / 이미지가 없는 라벨
  4. 클래스별 박스 개수 (train / val)

사용법:
  python3 check_labels.py                # 기본 dataset/ 검사
  python3 check_labels.py <데이터셋 폴더>  # images/, labels/ 가 들어 있는 폴더
"""
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
IMAGE_EXTS = {'.jpg', '.jpeg', '.png', '.bmp'}


def load_classes():
    """classes.txt 와 data.yaml 의 이름 목록을 읽고, 서로 다르면 종료"""
    txt = [line.strip() for line in (HERE / 'classes.txt').read_text().splitlines() if line.strip()]
    names = yaml.safe_load((HERE / 'data.yaml').read_text())['names']
    if isinstance(names, dict):
        names = [names[i] for i in sorted(names)]
    if txt != names:
        sys.exit(f'classes.txt 와 data.yaml 순서가 다릅니다\n  classes.txt: {txt}\n  data.yaml  : {names}')
    return names


def check_split(root, split, names):
    """한 묶음(train / val) 검사. 문제 수와 클래스별 박스 수를 돌려준다"""
    img_dir, lbl_dir = root / 'images' / split, root / 'labels' / split
    if not img_dir.is_dir():
        print(f'[{split}] {img_dir} 없음 -> 건너뜀')
        return 0, [0] * len(names)

    images = {p.stem for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS}
    labels = {p.stem: p for p in lbl_dir.glob('*.txt')} if lbl_dir.is_dir() else {}
    counts = [0] * len(names)
    problems = 0

    for stem in sorted(set(labels) - images):
        print(f'[{split}] 이미지 없는 라벨: {labels[stem].name}')
        problems += 1
    # 라벨 파일이 없는 이미지는 "아무것도 없는 배경"으로 학습되므로 에러는 아니지만 알려 준다
    no_label = sorted(images - set(labels))
    if no_label:
        print(f'[{split}] 라벨 없는 이미지 {len(no_label)}장 (배경 사진이면 정상): {", ".join(no_label[:5])}'
              + (' ...' if len(no_label) > 5 else ''))

    for stem, path in sorted(labels.items()):
        for n, line in enumerate(path.read_text().splitlines(), 1):
            if not line.strip():
                continue
            parts = line.split()
            where = f'[{split}] {path.name}:{n}'
            if len(parts) != 5:
                print(f'{where}: 값이 5개가 아님 ({line.strip()!r})')
                problems += 1
                continue
            try:
                cls = int(parts[0])
                box = [float(v) for v in parts[1:]]
            except ValueError:
                print(f'{where}: 숫자가 아님 ({line.strip()!r})')
                problems += 1
                continue
            if not 0 <= cls < len(names):
                print(f'{where}: 클래스 번호 {cls} 가 범위(0~{len(names) - 1}) 밖')
                problems += 1
                continue
            if not all(0.0 <= v <= 1.0 for v in box):
                print(f'{where}: 좌표가 0~1 밖 (픽셀 좌표로 저장했는지 확인) {box}')
                problems += 1
                continue
            counts[cls] += 1
    return problems, counts


def main():
    names = load_classes()
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / 'dataset'
    total_problems = 0
    table = {}
    for split in ('train', 'val'):
        problems, counts = check_split(root, split, names)
        total_problems += problems
        table[split] = counts

    print('\n번호  클래스              train    val')
    for i, name in enumerate(names):
        warn = '  <- 박스 없음' if table['train'][i] == 0 else ''
        print(f'{i:>3}  {name:<18} {table["train"][i]:>6} {table["val"][i]:>6}{warn}')
    print(f'\n문제 {total_problems}건')
    sys.exit(1 if total_problems else 0)


if __name__ == '__main__':
    main()
