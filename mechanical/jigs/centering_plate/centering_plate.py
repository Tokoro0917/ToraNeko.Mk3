"""機体を区画の真ん中に合わせる板（センタリング板）を生成する。

使い方:  python3 centering_plate.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに centering_plate.stl / .step

区画の床に置く、壁と壁のあいだにちょうど入る大きさの板。
真ん中に機体の形の穴があり、そのまわりの低い縁が基板の外形に当たって、
機体の中心（車軸の中点。基板の外形の中心と同じ）が区画の中心に来る。
壁センサの基準値を測るとき（左右の壁から同じ距離に置きたいとき）や、スタート位置に置くときに使う。

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
PLATE_T = 1.2          # 板の厚さ
RIM_W = 1.6            # 基板のまわりの縁の幅
RIM_H = 4.0            # 縁の高さ（床から）。基板の端（床から 2.36〜2.96mm）に当たる高さ
CORNER_CHAMFER = 1.0   # 板の四隅の面取り（区画の柱の角に当たらないように）

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
FLOOR_Z = -2.36                # 床の高さ（基板の下面から）
ROBOT_CENTER = (-13.4, -5.32)  # 機体の中心（基板の外形の中心 = 車軸の中点）
BOARD_W = 40.0                 # 基板の幅
BOARD_L = 64.0                 # 基板の長さ
BOARD_CORNER_R = 4.0           # 基板の四隅の丸み


def board_shape(margin, z0, z1):
    """基板の外形（四隅が丸い長方形）を margin だけ広げた形を、z0〜z1 で押し出す。

    前後のえぐれは無視し、外側の長方形だけを使う（四隅で位置が決まる）。
    """
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(ROBOT_CENTER[0], ROBOT_CENTER[1])
            .rect(BOARD_W + margin * 2, BOARD_L + margin * 2).extrude(z1 - z0)
            .edges("|Z").fillet(BOARD_CORNER_R + margin))


def plate():
    """区画に入る正方形の板。"""
    size = CELL_INNER - WALL_GAP * 2
    return (cq.Workplane("XY").workplane(offset=FLOOR_Z)
            .center(ROBOT_CENTER[0], ROBOT_CENTER[1])
            .rect(size, size).extrude(PLATE_T)
            .edges("|Z").chamfer(CORNER_CHAMFER))


def rim():
    """基板のまわりの縁（内側の穴はあとで centering_plate() でまとめてあける）。"""
    return board_shape(BOARD_GAP + RIM_W, FLOOR_Z, FLOOR_Z + RIM_H)


def marks():
    """上面に彫る「FRONT」と、中心を示す線（前後・左右）。"""
    x0, y0 = ROBOT_CENTER
    size = CELL_INNER - WALL_GAP * 2
    top_z = FLOOR_Z + PLATE_T
    text = (cq.Workplane("XY").workplane(offset=top_z - 0.4)
            .center(x0, y0 - BOARD_L / 2 - 6.0)
            .text("FRONT", 5.0, 1.0, halign="center", valign="center"))
    # 板の端から縁までの 4 本の線（区画の中心線に合っているかを目で確かめる用）
    line_w = 0.8
    reach = (size - BOARD_W) / 2 - RIM_W - 1.0
    left = cq.Workplane("XY").box(reach, line_w, 1.0).translate(
        (x0 + size / 2 - reach / 2, y0, top_z))
    right = cq.Workplane("XY").box(reach, line_w, 1.0).translate(
        (x0 - size / 2 + reach / 2, y0, top_z))
    reach_y = (size - BOARD_L) / 2 - RIM_W - 1.0
    rear = cq.Workplane("XY").box(line_w, reach_y, 1.0).translate(
        (x0, y0 + size / 2 - reach_y / 2, top_z))
    return text.union(left).union(right).union(rear)


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
    size = CELL_INNER - WALL_GAP * 2
    print(f"板の大きさ: {size:.1f} × {size:.1f} mm")
    print("done")
