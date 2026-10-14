"""バッテリーホルダーを生成する。

使い方:  python3 battery_holder.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに battery_holder.stl / .step

上部固定基板の上に載せ、バッテリー（12 × 25 × 6mm）2 本を横に並べて入れる箱。
吸引ファンモータマウントのネジ 2 本（ファンモータの左右）で上部固定基板と一緒に留める。

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面、上部固定基板の上面は z = 9.7
  前が -Y、機体の右が -X、機体の中心線は X = -13.4
STEP はこの座標のまま出力するので、Fusion 360 で機体に挿入するとそのまま正しい位置に来る。
STL はそのまま印刷できる向き（底面 z = 9.7 を 0 にずらしただけ）で出力する。

【バッテリーの高さについて】
STEP ではバッテリーが上部固定基板の上（z = 9.7）にじかに置かれているが、
上部固定基板の後ろ（Y > -1.82）では、モータマウントの上の出っ張りが z = 10.9 まで、
モータが z = 9.94 まであり、バッテリーと重なっている。
そこでこのホルダーでは、バッテリーの底を z = 11.1（出っ張りの 0.2mm 上）にしている。
"""
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 調整用パラメータ ------------------------------------------------------
BATT_W = 12.0          # バッテリー 1 本の幅（左右）。実物を測って変える
BATT_L = 25.0          # バッテリーの長さ（前後）
BATT_T = 6.0           # バッテリーの厚さ（上下）
CLEARANCE = 0.3        # バッテリーと壁のすきま（FDM なので大きめ）
WALL_T = 1.0           # 左右の壁の厚さ
SNAP = 0.3             # 壁の上端の、バッテリーを押さえるつめの出っ張り。0 にすると無し
FRONT_LIP_H = 2.0      # 前のストッパーの高さ（バッテリーの底から）
REAR_STOP_H = 2.0      # 後ろのストッパーの高さ（バッテリーの底から）
REAR_STOP_W = 4.0      # 後ろのストッパーの幅（左右の端だけ。真ん中は配線を出す口）
SCREW_HOLE_D = 1.7     # M1.4 の通し穴
SCREW_HEAD_D = 2.9     # M1.4 なべ頭を沈める座ぐり
SCREW_HEAD_DEPTH = 0.7 # 座ぐりの深さ（なべ頭の高さ 0.6 + 0.1）
HEAD_POCKET_D = 3.0    # 上部固定基板を留めている既存ネジの頭の逃げ（底面側）
HEAD_POCKET_H = 0.8

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
CENTER_X = -13.4               # 機体の中心線
UPPER_TOP_Z = 9.7              # 上部固定基板の上面
UPPER_REAR_Y = -1.82           # 上部固定基板の後ろの端
BATT_BOTTOM_Z = 11.1           # バッテリーの底（モータマウントの出っ張り 10.9 + 0.2）
BATT_FRONT_Y = -13.1           # バッテリーの前の端（ファンモータの後ろの端 -13.57 から 0.47mm）
FAN_MOTOR = (-13.4, -16.57)    # 吸引ファンモータ MK06（φ6）の中心
FAN_MOTOR_HOLE_D = 6.8         # ファンモータを通す穴
FAN_SCREWS = [(-17.9, -16.57), (-8.9, -16.57)]   # ファンモータマウントのネジ（これで留める）
UPPER_SCREWS = [(-25.3, -9.07), (-1.5, -9.07)]   # 上部固定基板をモータマウントに留めているネジ
# モータマウントの出っ張りとモータの上を空ける範囲（ここは底板を抜き、壁だけを渡す）
WINDOW_Y0 = -2.1
WINDOW_Y1 = 9.1

# ---- 計算で決まる寸法 ------------------------------------------------------
POCKET_HALF_W = BATT_W + CLEARANCE            # 中心線からポケットの内壁まで
OUTER_HALF_W = POCKET_HALF_W + WALL_T         # 中心線から外壁まで
POCKET_FRONT_Y = BATT_FRONT_Y - CLEARANCE
POCKET_REAR_Y = BATT_FRONT_Y + BATT_L + CLEARANCE
HOLDER_FRONT_Y = POCKET_FRONT_Y - WALL_T
HOLDER_REAR_Y = POCKET_REAR_Y + WALL_T
BATT_TOP_Z = BATT_BOTTOM_Z + BATT_T
WALL_TOP_Z = BATT_TOP_Z + 0.1 + SNAP * 2


def block(x0, x1, y0, y1, z0, z1):
    """x0〜x1、y0〜y1、z0〜z1 の直方体。"""
    return (cq.Workplane("XY")
            .box(x1 - x0, y1 - y0, z1 - z0, centered=False)
            .translate((x0, y0, z0)))


def cylinder(x, y, diameter, z0, z1):
    """中心 (x, y)、z0〜z1 の円柱。"""
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(x, y).circle(diameter / 2).extrude(z1 - z0))


def base():
    """底板。前（上部固定基板の上）と後ろの帯だけ。モータの上は抜く。"""
    left_x = CENTER_X + OUTER_HALF_W
    right_x = CENTER_X - OUTER_HALF_W
    front = block(right_x, left_x, HOLDER_FRONT_Y, WINDOW_Y0, UPPER_TOP_Z, BATT_BOTTOM_Z)
    rear = block(right_x, left_x, WINDOW_Y1, HOLDER_REAR_Y, UPPER_TOP_Z, BATT_BOTTOM_Z)
    return front.union(rear)


def side_walls():
    """左右の壁。窓の範囲ではバッテリーの底より下を抜く（モータマウントの出っ張りをよける）。"""
    walls = None
    for sign in (-1, 1):
        x_in = CENTER_X + sign * POCKET_HALF_W
        x_out = CENTER_X + sign * OUTER_HALF_W
        wall = block(min(x_in, x_out), max(x_in, x_out),
                     HOLDER_FRONT_Y, HOLDER_REAR_Y, UPPER_TOP_Z, WALL_TOP_Z)
        if walls is None:
            walls = wall
        else:
            walls = walls.union(wall)
    window = block(CENTER_X - 20, CENTER_X + 20, WINDOW_Y0, WINDOW_Y1, UPPER_TOP_Z - 1, BATT_BOTTOM_Z)
    return walls.cut(window)


def snap_lips():
    """壁の上端の内側に付けるつめ。下面は平らでバッテリーの上面を押さえ、上面は斜めで差し込みやすくする。"""
    if SNAP <= 0:
        return None
    lip_y0 = -8.0
    lip_y1 = 6.0
    z0 = BATT_TOP_Z + 0.1
    lips = None
    for sign in (-1, 1):
        x_wall = CENTER_X + sign * POCKET_HALF_W
        x_tip = x_wall - sign * SNAP
        # 断面（x, z）の三角形を Y 方向に押し出す
        profile = [(x_wall, z0), (x_tip, z0), (x_wall, z0 + SNAP * 2)]
        lip = (cq.Workplane("XZ", origin=(0, lip_y1, 0))
               .polyline(profile).close().extrude(lip_y1 - lip_y0))
        if lips is None:
            lips = lip
        else:
            lips = lips.union(lip)
    return lips


def stoppers():
    """前のストッパー（ファンモータの左右）と、後ろのストッパー（左右の端）。"""
    left_x = CENTER_X + OUTER_HALF_W
    right_x = CENTER_X - OUTER_HALF_W
    gap_half = FAN_MOTOR_HOLE_D / 2 + 0.4   # ファンモータの前を空ける
    front_top = BATT_BOTTOM_Z + FRONT_LIP_H
    front_r = block(right_x, CENTER_X - gap_half, HOLDER_FRONT_Y, POCKET_FRONT_Y, UPPER_TOP_Z, front_top)
    front_l = block(CENTER_X + gap_half, left_x, HOLDER_FRONT_Y, POCKET_FRONT_Y, UPPER_TOP_Z, front_top)
    rear_top = BATT_BOTTOM_Z + REAR_STOP_H
    rear_r = block(right_x, right_x + REAR_STOP_W, POCKET_REAR_Y, HOLDER_REAR_Y, UPPER_TOP_Z, rear_top)
    rear_l = block(left_x - REAR_STOP_W, left_x, POCKET_REAR_Y, HOLDER_REAR_Y, UPPER_TOP_Z, rear_top)
    return front_r.union(front_l).union(rear_r).union(rear_l)


def screw_plate():
    """ファンモータの左右のネジで留める板。ファンモータを通す穴をあける。"""
    x0 = FAN_SCREWS[0][0] - 2.1
    x1 = FAN_SCREWS[1][0] + 2.1
    y0 = FAN_MOTOR[1] - 2.0
    return block(x0, x1, y0, HOLDER_FRONT_Y, UPPER_TOP_Z, BATT_BOTTOM_Z)


def cut_holes(body):
    """ファンモータの穴、留めネジの穴と座ぐり、既存ネジの頭の逃げをあける。"""
    body = body.cut(cylinder(FAN_MOTOR[0], FAN_MOTOR[1], FAN_MOTOR_HOLE_D, 0, 30))
    for x, y in FAN_SCREWS:
        body = body.cut(cylinder(x, y, SCREW_HOLE_D, 0, 30))
        body = body.cut(cylinder(x, y, SCREW_HEAD_D, BATT_BOTTOM_Z - SCREW_HEAD_DEPTH, 30))
    for x, y in UPPER_SCREWS:
        body = body.cut(cylinder(x, y, HEAD_POCKET_D, 0, UPPER_TOP_Z + HEAD_POCKET_H))
    return body


def holder():
    """バッテリーホルダー一式。"""
    body = base().union(side_walls()).union(stoppers()).union(screw_plate())
    lips = snap_lips()
    if lips is not None:
        body = body.union(lips)
    return cut_holes(body)


def export(body, name):
    """STEP は機体の座標のまま、STL は底面を z = 0 にずらして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    bb = body.val().BoundingBox()
    moved = body.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(moved, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    export(holder(), "battery_holder")
    print(f"バッテリーの底 z = {BATT_BOTTOM_Z}（STEP の位置から {BATT_BOTTOM_Z - UPPER_TOP_Z:.1f}mm 上）")
    print("done")
