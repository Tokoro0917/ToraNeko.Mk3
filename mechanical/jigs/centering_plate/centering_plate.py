"""機体を区画の決まった位置に置く板（センタリング板）を生成する。

使い方:  python3 centering_plate.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに
         centering_plate.stl / .step          まっすぐ版（機体の中心を区画の中心に、まっすぐ前向きに置く）
         centering_plate_diag_R.stl / .step   斜め版・右（機体の中心を区画の辺の中点に、右斜め 45° 向きに置く）
         centering_plate_diag_L.stl / .step   斜め版・左（右と左右反対。左斜め 45° 向き）

どれも区画の床に置き、左右の壁と後ろの壁に当てる板。
機体の後ろ側の 2 つの角を L 字の縁で受けて、機体の中心（車軸の中点。基板の外形の中心と同じ）の位置と向きを決める。
機体の前には何もないので、板を置いたまま機体をスタートさせられる（直進やターンの調整に使える）。
その場旋回をするときだけは、板を持ち上げて外してからスタートする。

【斜め版の置き方】
斜めの動き（斜め入りの終わり、V90、斜め出の始め）は、区画の辺の中点を 45° の向きで通る。
斜め版は、3 方に壁がある区画（スタート区画と同じ形）に置き、壁のない辺の中点に機体の中心を合わせる。
このとき機体の後ろ半分はその区画に、前半分は壁のない辺の向こうの区画に入り、壁にも柱にも当たらない。

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面、床は z = -2.36、前が -Y、機体の右が -X、機体の中心は (X, Y) = (-13.4, -5.32)
板はどれも、機体の中心を基準に作る（斜め版は、区画のほうを 45° 回して作る）。
STEP はこの座標のまま（機体を置いた状態の位置で）出力する。
STL はそのまま印刷できる向き（底面を z = 0 にずらしただけ）で出力する。
"""
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 調整用パラメータ ------------------------------------------------------
CELL_INNER = 84.0      # 壁と壁のあいだの距離（ハーフサイズ: 区画 90mm − 壁の厚さ 6mm）。実際の迷路を測って変える
CELL_PITCH = 90.0      # 区画の大きさ（柱の中心から次の柱の中心まで）。斜め版で使う
WALL_GAP = 0.3         # 板と壁のすきま（片側）
BOARD_GAP = 0.3        # 縁と基板の外形のすきま
PLATE_T = 2.0          # 板の厚さ
RIM_W = 1.6            # 基板の角を受ける縁の幅
RIM_H = 4.0            # 縁の高さ（床から）。基板の端（床から 2.36〜2.96mm）に当たる高さ
RIM_LEN = 10.0         # 縁が基板の横を受ける長さ（基板の後ろの端から前へ）
ARM_FRONT = -10.0      # まっすぐ版の左右の腕の前の端（機体の中心から。マイナスが前）
CORNER_CHAMFER = 1.0   # 板の角の面取り（区画の柱の角に当たらないように）

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
FLOOR_Z = -2.36                # 床の高さ（基板の下面から）
ROBOT_CENTER = (-13.4, -5.32)  # 機体の中心（基板の外形の中心 = 車軸の中点）
BOARD_W = 40.0                 # 基板の幅
BOARD_L = 64.0                 # 基板の長さ
BOARD_CORNER_R = 4.0           # 基板の四隅の丸み

# ---- 計算で決まる寸法 ------------------------------------------------------
HALF_CELL = CELL_INNER / 2 - WALL_GAP         # 区画の中心から板の外側の端まで
BOARD_REAR = BOARD_L / 2                      # 機体の中心から基板の後ろの端まで
# 斜め版で、機体の中心（辺の中点）から、後ろ半分が入る区画の中心までのずれ。
# 区画の中心は辺の中点から半区画（45mm）離れていて、機体から見ると後ろ斜め 45° の方向にある。
# そのため、左右と前後にそれぞれ 45 / √2 = 31.8mm ずれる
DIAG_OFFSET = CELL_PITCH / 2 / math.sqrt(2)


def at(u, v, z):
    """機体の中心から左 u、後ろ v の位置を STEP 座標にする（u は左がプラス、v は後ろがプラス）。"""
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


def straight_plate():
    """まっすぐ版の板（機体の形を抜く前の長方形）。区画の後ろ半分に入る。"""
    length = HALF_CELL - ARM_FRONT
    center = at(0, (HALF_CELL + ARM_FRONT) / 2, FLOOR_Z)
    return (cq.Workplane("XY").workplane(offset=FLOOR_Z)
            .center(center[0], center[1])
            .rect(HALF_CELL * 2, length).extrude(PLATE_T)
            .edges("|Z").chamfer(CORNER_CHAMFER))


def diagonal_plate(side):
    """斜め版の板（機体の形を抜く前の正方形）。機体の後ろ半分が入る区画いっぱいの大きさ。

    side = +1 で右斜め向き（区画は機体の右後ろ）、-1 で左斜め向き（区画は機体の左後ろ）。
    STEP 座標では機体の右が -X なので、右後ろは u がマイナスの方向になる。
    """
    center = at(-side * DIAG_OFFSET, DIAG_OFFSET, FLOOR_Z)
    square = (cq.Workplane("XY").workplane(offset=FLOOR_Z)
              .center(center[0], center[1])
              .rect(HALF_CELL * 2, HALF_CELL * 2).extrude(PLATE_T)
              .edges("|Z").chamfer(CORNER_CHAMFER))
    # 区画の中心のまわりに 45° 回す（区画の辺が機体に対して斜めになる）
    axis_end = (center[0], center[1], center[2] + 1)
    return square.rotate(center, axis_end, 45)


def rim():
    """基板の後ろの 2 つの角を受ける L 字の縁（後ろの端と、横の後ろ RIM_LEN だけ）。"""
    outer = board_shape(BOARD_GAP + RIM_W, FLOOR_Z, FLOOR_Z + RIM_H)
    keep_front_v = BOARD_REAR - RIM_LEN
    keep_rear_v = BOARD_REAR + 10.0
    keep = (cq.Workplane("XY")
            .box(CELL_INNER, keep_rear_v - keep_front_v, 30)
            .translate(at(0, (keep_rear_v + keep_front_v) / 2, 0)))
    return outer.intersect(keep)


def center_line():
    """機体の後ろに彫る、機体の中心線の延長（向きを目で確かめる用）。"""
    top_z = FLOOR_Z + PLATE_T
    rear_start = BOARD_REAR + BOARD_GAP + RIM_W + 0.5
    return cq.Workplane("XY").box(0.8, 6.0, 1.0).translate(at(0, rear_start + 3.0, top_z))


def straight_marks():
    """まっすぐ版の上面に彫る「FRONT」と前向きの線（左右の腕）、中心線（後ろ）。"""
    top_z = FLOOR_Z + PLATE_T
    arm_u = (HALF_CELL + BOARD_W / 2 + BOARD_GAP + RIM_W) / 2   # 腕の幅の真ん中
    result = center_line()
    for sign in (-1, 1):
        position = at(sign * arm_u, ARM_FRONT + 4.0, 0)
        text = (cq.Workplane("XY").workplane(offset=top_z - 0.4)
                .center(position[0], position[1])
                .text("FRONT", 3.5, 1.0, halign="center", valign="center"))
        line = cq.Workplane("XY").box(0.8, 20.0, 1.0).translate(
            at(sign * arm_u, ARM_FRONT + 17.0, top_z))
        result = result.union(text).union(line)
    return result


def diagonal_marks(side):
    """斜め版の上面に彫る「R45」か「L45」と、中心線（後ろ）。"""
    top_z = FLOOR_Z + PLATE_T
    if side > 0:
        name = "R45"
    else:
        name = "L45"
    position = at(-side * (DIAG_OFFSET + 10.0), DIAG_OFFSET, 0)
    text = (cq.Workplane("XY").workplane(offset=top_z - 0.4)
            .center(position[0], position[1])
            .text(name, 5.0, 1.0, halign="center", valign="center"))
    return text.union(center_line())


def finish(plate, carved):
    """板に縁を足し、機体の形を抜き、文字を彫る。"""
    body = plate.union(rim())
    hole = board_shape(BOARD_GAP, FLOOR_Z - 1, FLOOR_Z + RIM_H + 1)
    return body.cut(hole).cut(carved)


def centering_plate():
    """まっすぐ版のセンタリング板。"""
    return finish(straight_plate(), straight_marks())


def diagonal_centering_plate(side):
    """斜め版のセンタリング板。side = +1 で右斜め、-1 で左斜め。"""
    return finish(diagonal_plate(side), diagonal_marks(side))


def export(body, name):
    """STEP は機体の座標のまま、STL は底面を z = 0 にずらして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    bb = body.val().BoundingBox()
    moved = body.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(moved, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    export(centering_plate(), "centering_plate")
    export(diagonal_centering_plate(+1), "centering_plate_diag_R")
    export(diagonal_centering_plate(-1), "centering_plate_diag_L")
    print(f"まっすぐ版: 幅 {HALF_CELL * 2:.1f} × 奥行き {HALF_CELL - ARM_FRONT:.1f} mm")
    print(f"斜め版: {HALF_CELL * 2:.1f} mm 角を 45° 回したもの")
    print("done")
