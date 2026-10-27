#!/usr/bin/env python3
"""
wall_table.py
logs/ に保存したセンサ値の記録（SENSOR）から、壁までの距離の表（ctrl/wall_dist.c）に貼る行を作る。

使い方（software/main-controller/ で）:
    python3 tools/wall_table.py logs/*_sensor_*.txt

■ メモの書き方（ビューアの「メモ」に書く。ファイルの 1 行目の「# memo: …」になる）
    R 64mm   … 右のセンサを、右の壁まで 64mm で測った
    L 40mm   … 左のセンサを、左の壁まで 40mm で測った
    F 52mm   … 前のセンサ（左前と右前の両方）を、前の壁まで 52mm で測った
    FL 52mm・FR 52mm … 左前だけ・右前だけ
  メモがこの形でないファイル（区画のまん中で測ったものなど）は使わない。
  同じセンサ・同じ距離のファイルが何個かあれば、平均する。

■ 出るもの
  センサごとに、wall_dist.c の「#define ○○_POINT_COUNT」と 2 つの表（距離・値）の行。
  そのまま wall_dist.c の同じ名前の行と置きかえる。
  距離が遠くなるほど値が小さくなっていないところがあれば、「注意」を出す（置き方や測り方を見直す）。
"""
import re
import sys

# メモの形：センサの名前、空白（なくてもよい）、距離、mm
MEMO_PATTERN = re.compile(r"^(FL|FR|F|L|R)\s*([0-9]+(?:\.[0-9]+)?)\s*mm$", re.IGNORECASE)

# wall_dist.c の表の名前（センサの名前 → 表の名前の頭）
TABLE_NAMES = {"FL": "FRONT_LEFT", "L": "LEFT", "R": "RIGHT", "FR": "FRONT_RIGHT"}


def read_sensor_blocks(path):
    """ファイルの中の SENSOR を、(メモ, {センサの名前: 平均}) の並びにして返す"""
    blocks = []
    memo = ""
    averages = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            line = raw_line.strip()
            if line.startswith("# memo:"):
                memo = line[len("# memo:"):].strip()
            elif line.startswith("#BEGIN SENSOR"):
                averages = {}
            elif line.startswith("#END SENSOR"):
                if averages is not None:
                    blocks.append((memo, averages))
                averages = None
                memo = ""
            elif averages is not None:
                fields = line.split(",")
                if len(fields) >= 2 and fields[0] in TABLE_NAMES:
                    averages[fields[0]] = float(fields[1])
    return blocks


def collect(paths):
    """全部のファイルから、{センサの名前: {距離: [平均, ...]}} を作る"""
    points = {name: {} for name in TABLE_NAMES}
    for path in paths:
        for memo, averages in read_sensor_blocks(path):
            match = MEMO_PATTERN.match(memo)
            if not match:
                print(f"// 使わない（メモの形が違う）: {path} [{memo}]", file=sys.stderr)
                continue
            target = match.group(1).upper()
            distance_mm = float(match.group(2))
            if target == "F":
                sensors = ["FL", "FR"]
            else:
                sensors = [target]
            for name in sensors:
                if name in averages:
                    points[name].setdefault(distance_mm, []).append(averages[name])
    return points


def format_number(value):
    """52.0 は 52、52.5 は 52.5 と書く"""
    if value == int(value):
        return str(int(value))
    return str(value)


def print_table(name, distance_points):
    """1 つのセンサの表の行を出す"""
    table = TABLE_NAMES[name]
    distances = sorted(distance_points)
    values = [round(sum(distance_points[d]) / len(distance_points[d])) for d in distances]
    print(f"/* ---- {name} の表（測った点 {len(distances)} 個） ---- */")
    print(f"#define {table}_POINT_COUNT {len(distances)}")
    print(f"static const float {table}_MM[{table}_POINT_COUNT] = {{{', '.join(format_number(d) for d in distances)}}};")
    print(f"static const int32_t {table}_VALUE[{table}_POINT_COUNT] = {{{', '.join(str(v) for v in values)}}};")
    for i in range(len(values) - 1):
        if values[i + 1] > values[i]:
            print(f"/* 注意: {format_number(distances[i + 1])}mm の値 {values[i + 1]} が、"
                  f"それより近い {format_number(distances[i])}mm の値 {values[i]} より大きい */")
    print()


def main():
    if len(sys.argv) < 2:
        print("使い方: python3 tools/wall_table.py logs/*_sensor_*.txt", file=sys.stderr)
        return 1
    points = collect(sys.argv[1:])
    for name in ["FL", "L", "R", "FR"]:
        if len(points[name]) < 2:
            print(f"/* {name}: 測った点が {len(points[name])} 個しかないので、表を作らない（2 個以上いる） */\n")
            continue
        print_table(name, points[name])
    return 0


if __name__ == "__main__":
    sys.exit(main())
