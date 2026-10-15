"""機体を区画の決まった位置に置く板（センタリング板）を生成する。

使い方:  python3 centering_plate.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに
         centering_plate.stl / .step          まっすぐ版（機体の中心を区画の中心に、まっすぐ前向きに置く）
         centering_plate_diag_R.stl / .step   斜め版・右（機体の中心を区画の辺の中点に、右斜め 45° 向きに置く）
         centering_plate_diag_L.stl / .step   斜め版・左（右と左右反対。左斜め 45° 向き）

どれも区画の床に置き、壁に当てて使う。機体の後ろの 2 つの角を L 字の縁で受けて、
機体の中心（車軸の中点。基板の外形の中心と同じ）の位置と向きを決める。
機体の前には何もないので、板を置いたまま機体をスタートさせられる（直進やターンの調整に使える）。
その場旋回をするときだけは、板を持ち上げて外してからスタートする。

形は、位置決めに要るところだけを残した細い枠にしてある。
  まっすぐ版: 機体の後ろと後ろの壁のあいだに入る帯。両端が左右の壁に、背中が後ろの壁に当たる
  斜め版:     機体の後ろの辺に沿う棒と、横の壁に沿う棒をつないだ形。
              横の壁に沿う棒が壁に沿って当たり（向きと左右の位置）、その端が奥の壁に当たる（前後の位置）

【斜め版の置き方】
斜めの動き（斜め入りの終わり、V90、斜め出の始め）は、区画の辺の中点を 45° の向きで通る。
斜め版は、3 方に壁がある区画（区画 B。スタート区画と同じ形）に置き、壁のない辺の中点に機体の中心を合わせる。
このとき機体の後ろ半分は区画 B に、前半分は壁のない辺の向こうの区画（区画 A）に入り、壁にも柱にも当たらない。

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面、床は z = -2.36、前が -Y、機体の右が -X、機体の中心は (X, Y) = (-13.4, -5.32)
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
PLATE_T = 3.0          # 板の厚さ（細いので、曲がらないように厚めにする）
RIM_W = 1.6            # 基板の角を受ける縁の幅
RIM_H = 4.0            # 縁の高さ（床から）。基板の端（床から 2.36〜2.96mm）に当たる高さ
RIM_LEN = 10.0         # 縁が基板の横を受ける長さ（基板の後ろの端から前へ）
REAR_BAR_W = 6.0       # 斜め版の、機体の後ろの辺に沿う棒の幅（縁の後ろから）
SIDE_BAR_W = 8.0       # 斜め版の、横の壁に沿う棒の幅

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
FLOOR_Z = -2.36                # 床の高さ（基板の下面から）
ROBOT_CENTER = (-13.4, -5.32)  # 機体の中心（基板の外形の中心 = 車軸の中点）
BOARD_W = 40.0                 # 基板の幅
BOARD_L = 64.0                 # 基板の長さ
BOARD_CORNER_R = 4.0           # 基板の四隅の丸み

# ---- 計算で決まる寸法 ------------------------------------------------------
HALF_CELL = CELL_INNER / 2 - WALL_GAP         # 区画の中心から板の外側の端まで
BOARD_REAR = BOARD_L / 2                      # 機体の中心から基板の後ろの端まで
RIM_INNER_V = BOARD_REAR + BOARD_GAP          # 機体の中心から縁の内側（基板に当たる面）まで
RIM_OUTER_U = BOARD_W / 2 + BOARD_GAP + RIM_W # 機体の中心から縁の外側（左右）まで
# 斜め版の区画 B の範囲（区画の座標。原点が辺の中点、x が東、y が北）
CELL_B_WEST = -HALF_CELL
CELL_B_EAST = HALF_CELL
CELL_B_SOUTH = -CELL_PITCH / 2 - HALF_CELL
CELL_B_NORTH = -CELL_PITCH / 2 + HALF_CELL


def at(u, v, z):
    """機体の中心から左 u、後ろ v の位置を STEP 座標にする（u は左がプラス、v は後ろがプラス）。"""
    return (ROBOT_CENTER[0] + u, ROBOT_CENTER[1] + v, z)


def cell_to_step(x, y):
    """斜め版（右）で、区画の座標 (x, y) を STEP 座標の (X, Y) にする。

    区画の座標: 原点が辺の中点（= 機体の中心）、x が東、y が北。区画 B は y < 0 の側。
    右斜め版では機体の前が北東を向くので、機体から見て
      左 u = (−x + y) / √2、後ろ v = (−x − y) / √2
    になる。
    """
    u = (-x + y) / math.sqrt(2)
    v = (-x - y) / math.sqrt(2)
    position = at(u, v, 0)
    return (position[0], position[1])


def prism(points, z0, z1):
    """STEP 座標の (X, Y) の点を順に結んだ形を、z0〜z1 で押し出す。"""
    return (cq.Workplane("XY").workplane(offset=z0)
            .polyline(points).close().extrude(z1 - z0))


def cell_rect(x0, x1, y0, y1, z0, z1):
    """斜め版（右）で、区画の座標で x0〜x1、y0〜y1 の長方形を z0〜z1 で押し出す（機体から見ると 45° 回っている）。"""
    points = [cell_to_step(x0, y0), cell_to_step(x1, y0), cell_to_step(x1, y1), cell_to_step(x0, y1)]
    return prism(points, z0, z1)


def robot_rect(u0, u1, v0, v1, z0, z1):
    """機体の座標で u0〜u1、v0〜v1 の直方体を z0〜z1 で作る。"""
    a = at(min(u0, u1), min(v0, v1), z0)
    return (cq.Workplane("XY")
            .box(abs(u1 - u0), abs(v1 - v0), z1 - z0, centered=False)
            .translate(a))


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


def rim():
    """基板の後ろの 2 つの角を受ける L 字の縁（後ろの端と、横の後ろ RIM_LEN だけ）。"""
    outer = board_shape(BOARD_GAP + RIM_W, FLOOR_Z, FLOOR_Z + RIM_H)
    keep = robot_rect(-50, 50, BOARD_REAR - RIM_LEN, BOARD_REAR + 10, -10, 10)
    return outer.intersect(keep)


def straight_frame():
    """まっすぐ版の枠: 機体の後ろの縁から後ろの壁までの帯（左右は壁まで）と、縁。"""
    band = robot_rect(-HALF_CELL, HALF_CELL, RIM_INNER_V, HALF_CELL, FLOOR_Z, FLOOR_Z + PLATE_T)
    return band.union(rim())


def diagonal_frame():
    """斜め版（右）の枠: 機体の後ろの辺に沿う棒、区画 B の西の壁に沿う棒、縁。区画 B から出る所は切る。"""
    rear_bar = robot_rect(-RIM_OUTER_U, RIM_OUTER_U,
                          RIM_INNER_V, RIM_INNER_V + RIM_W + REAR_BAR_W,
                          FLOOR_Z, FLOOR_Z + PLATE_T)
    side_bar = cell_rect(CELL_B_WEST, CELL_B_WEST + SIDE_BAR_W, CELL_B_SOUTH, CELL_B_NORTH,
                         FLOOR_Z, FLOOR_Z + PLATE_T)
    frame = rear_bar.union(side_bar).union(rim())
    cell_b = cell_rect(CELL_B_WEST, CELL_B_EAST, CELL_B_SOUTH, CELL_B_NORTH,
                       FLOOR_Z - 1, FLOOR_Z + RIM_H + 1)
    return frame.intersect(cell_b)


def cut_board(frame):
    """機体の形（前へ抜ける範囲も含む）を抜く。"""
    return frame.cut(board_shape(BOARD_GAP, FLOOR_Z - 1, FLOOR_Z + RIM_H + 1))


def carve_text(body, text, x, y, size, angle_deg):
    """上面の (x, y) に文字を 0.4mm の深さで彫る。文字は (x, y) のまわりに angle_deg だけ回す。"""
    top_z = FLOOR_Z + PLATE_T
    letters = (cq.Workplane("XY").workplane(offset=top_z - 0.4)
               .center(x, y)
               .text(text, size, 1.0, halign="center", valign="center"))
    letters = letters.rotate((x, y, 0), (x, y, 1), angle_deg)
    return body.cut(letters)


def centering_plate():
    """まっすぐ版のセンタリング板。帯の左右に「FRONT」と彫る。

    文字はそのままだと上が +Y（機体の後ろ）を向くので、180° 回して、読める向きが機体の前になるようにする。
    """
    body = cut_board(straight_frame())
    text_v = (RIM_INNER_V + RIM_W + HALF_CELL) / 2
    for sign in (-1, 1):
        position = at(sign * 31.0, text_v, 0)
        body = carve_text(body, "FRONT", position[0], position[1], 3.5, 180)
    return body


def diagonal_centering_plate_right():
    """斜め版・右のセンタリング板。西の壁に沿う棒に「R」と彫る。"""
    body = cut_board(diagonal_frame())
    position = cell_to_step(CELL_B_WEST + SIDE_BAR_W / 2, -CELL_PITCH / 2)
    return carve_text(body, "R", position[0], position[1], 5.0, 0)


def diagonal_centering_plate_left():
    """斜め版・左のセンタリング板。右を、機体の中心線で左右反対にする（文字は反対にせず「L」を彫る）。"""
    body = cut_board(diagonal_frame())
    body = body.mirror("YZ", basePointVector=(ROBOT_CENTER[0], 0, 0))
    position = cell_to_step(CELL_B_WEST + SIDE_BAR_W / 2, -CELL_PITCH / 2)
    mirrored_x = 2 * ROBOT_CENTER[0] - position[0]
    return carve_text(body, "L", mirrored_x, position[1], 5.0, 0)


def export(body, name):
    """STEP は機体の座標のまま、STL は底面を z = 0 にずらして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    bb = body.val().BoundingBox()
    moved = body.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(moved, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    export(centering_plate(), "centering_plate")
    export(diagonal_centering_plate_right(), "centering_plate_diag_R")
    export(diagonal_centering_plate_left(), "centering_plate_diag_L")
    print("done")
