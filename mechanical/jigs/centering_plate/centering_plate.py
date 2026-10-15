"""機体を区画の真ん中に合わせる板（センタリング板）を生成する。

使い方:  python3 centering_plate.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに centering_plate.stl / .step

区画の床に置く、前が開いた U 字の板。左右の壁と後ろの壁に当てて置く。
機体の後ろ側の 2 つの角を L 字の縁で受けて、機体の中心（車軸の中点。基板の外形の中心と同じ）を
区画の中心に合わせる。前には何もないので、板を置いたまま機体をスタートさせられる
（直進やターンの調整に使える）。その場旋回をするときだけは、板を持ち上げて外してからスタートする。

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面、床は z = -2.36、前が -Y、機体の中心は (X, Y) = (-13.4, -5.32)
STEP はこの座標のまま（機体を入れた状態の位置で）出力する。
STL はそのまま印刷できる向き（底面を z = 0 にずらしただけ）で出力する。
"""
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 調整用パラメータ ------------------------------------------------------
CELL_INNER = 84.0      # 壁と壁のあいだの距離（ハーフサイズ: 区画 90mm − 壁の厚さ 6mm）。実際の迷路を測って変える
WALL_GAP = 0.3         # 板と壁のすきま（片側）
BOARD_GAP = 0.3        # 縁と基板の外形のすきま
PLATE_T = 2.0          # 板の厚さ
RIM_W = 1.6            # 基板の角を受ける縁の幅
RIM_H = 4.0            # 縁の高さ（床から）。基板の端（床から 2.36〜2.96mm）に当たる高さ
RIM_LEN = 10.0         # 縁が基板の横を受ける長さ（基板の後ろの端から前へ）
ARM_FRONT = -10.0      # 左右の腕の前の端（機体の中心から。マイナスが前）
CORNER_CHAMFER = 1.0   # 板の角の面取り（区画の柱の角に当たらないように）

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
FLOOR_Z = -2.36                # 床の高さ（基板の下面から）
ROBOT_CENTER = (-13.4, -5.32)  # 機体の中心（基板の外形の中心 = 車軸の中点）
BOARD_W = 40.0                 # 基板の幅
BOARD_L = 64.0                 # 基板の長さ
BOARD_CORNER_R = 4.0           # 基板の四隅の丸み

# ---- 計算で決まる寸法（機体の中心からの距離。前後は後ろがプラス）---------
HALF_CELL = CELL_INNER / 2 - WALL_GAP         # 中心から板の外側の端まで
BOARD_REAR = BOARD_L / 2                      # 中心から基板の後ろの端まで


def at(u, v, z):
    """機体の中心から左右 u、前後 v（後ろがプラス）の位置を、STEP 座標にする。"""
    return (ROBOT_CENTER[0] + u, ROBOT_CENTER[1] + v, z)


def board_shape(margin, z0, z1):
    """基板の外形（四隅が丸い長方形）を margin だけ広げた形を、z0〜z1 で押し出す。

    前を長く伸ばしてあるので、前が開いた形になる（機体が前に抜けられる）。
    前後のえぐれは無視し、外側の長方形だけを使う（後ろの 2 つの角で位置が決まる）。
    """
    width = BOARD_W + margin * 2
    rear_v = BOARD_REAR + margin
    front_v = -100.0
    center = at(0, (rear_v + front_v) / 2, z0)
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(center[0], center[1])
            .rect(width, rear_v - front_v).extrude(z1 - z0)
            .edges("|Z").fillet(BOARD_CORNER_R + margin))


def plate():
    """区画の後ろ半分に入る板（U 字にする前の長方形）。"""
    length = HALF_CELL - ARM_FRONT
    center = at(0, (HALF_CELL + ARM_FRONT) / 2, FLOOR_Z)
    return (cq.Workplane("XY").workplane(offset=FLOOR_Z)
            .center(center[0], center[1])
            .rect(HALF_CELL * 2, length).extrude(PLATE_T)
            .edges("|Z").chamfer(CORNER_CHAMFER))


def rim():
    """基板の後ろの 2 つの角を受ける L 字の縁（後ろの端と、横の後ろ RIM_LEN だけ）。"""
    outer = board_shape(BOARD_GAP + RIM_W, FLOOR_Z, FLOOR_Z + RIM_H)
    keep_front_v = BOARD_REAR - RIM_LEN
    keep = (cq.Workplane("XY")
            .box(CELL_INNER, HALF_CELL - keep_front_v, 30)
            .translate(at(0, (HALF_CELL + keep_front_v) / 2, 0)))
    return outer.intersect(keep)


def marks():
    """上面に彫る「FRONT」と矢印の線（左右の腕）、中心線（後ろ）。"""
    top_z = FLOOR_Z + PLATE_T
    arm_u = (HALF_CELL + BOARD_W / 2 + BOARD_GAP + RIM_W) / 2   # 腕の幅の真ん中
    result = None
    for sign in (-1, 1):
        text = (cq.Workplane("XY").workplane(offset=top_z - 0.4)
                .center(at(sign * arm_u, 0, 0)[0], at(0, ARM_FRONT + 4.0, 0)[1])
                .text("FRONT", 3.5, 1.0, halign="center", valign="center"))
        line = cq.Workplane("XY").box(0.8, 20.0, 1.0).translate(
            at(sign * arm_u, ARM_FRONT + 17.0, top_z))
        if result is None:
            result = text.union(line)
        else:
            result = result.union(text).union(line)
    rear_start = BOARD_REAR + BOARD_GAP + RIM_W + 0.5
    center_line = cq.Workplane("XY").box(0.8, HALF_CELL - rear_start - 1.0, 1.0).translate(
        at(0, (HALF_CELL + rear_start) / 2, top_z))
    return result.union(center_line)


def centering_plate():
    """センタリング板一式。"""
    body = plate().union(rim())
    hole = board_shape(BOARD_GAP, FLOOR_Z - 1, FLOOR_Z + RIM_H + 1)
    body = body.cut(hole)
    return body.cut(marks())


def export(body, name):
    """STEP は機体の座標のまま、STL は底面を z = 0 にずらして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    bb = body.val().BoundingBox()
    moved = body.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(moved, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    export(centering_plate(), "centering_plate")
    print(f"板の大きさ: 幅 {HALF_CELL * 2:.1f} × 奥行き {HALF_CELL - ARM_FRONT:.1f} mm")
    print("done")
