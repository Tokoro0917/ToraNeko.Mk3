"""吸引ファン用スカート枠と、前後のスライダーを生成する。

使い方:  python3 skirt_slider.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに
         skirt_frame.stl / .step   スカート枠（前後のスライダーの出っ張り付き）
         slider_front.stl / .step  スカートを付けないときの前スライダー
         slider_rear.stl / .step   スカートを付けないときの後ろスライダー

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面（STEP の基板は z = 0〜0.6）、床は z = -2.36（タイヤの下端）
  前が -Y、機体の右が -X、機体の中心線は X = -13.4、車軸は Y = -5.32
  KiCad 座標 (xk, yk) からは X = 96.5 - xk、Y = yk - 71.37 で変換できる
STEP はこの座標のまま出力するので、Fusion 360 で機体に挿入するとそのまま正しい位置に来る。
STL は印刷の向き（基板に当たる面が下）に裏返して出力する。
"""
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 調整用パラメータ ------------------------------------------------------
FLOOR_Z = -2.36        # 床の高さ（基板下面からタイヤ下端まで）。タイヤを削ったら変える
FRAME_H = 1.6          # スカート枠の高さ。枠の下端と床のすきまは 2.36 - 1.6 = 0.76mm
WALL_T = 1.2           # 枠の壁の厚さ（0.4mm ノズルで 3 周）
SLIDER_GAP = 0.5       # スライダーの下端と床のすきま
SLIDER_R = 1.8         # スライダーの出っ張りの半径
SLIDER_EDGE_R = 0.4    # スライダー下端の角の丸み
SCREW_PLATE_T = 0.8    # ネジが通る部分の厚さ（ネジはこの分だけ長いものに替える）
SCREW_HOLE_D = 1.7     # M1.4 の通し穴（FDM は穴が小さめに出るので 1.7）
SCREW_HEAD_D = 2.9     # M1.4 なべ頭（φ2.5）を沈める座ぐり
POST_BOSS_R = 1.9      # 支柱のネジまわりの太い部分の半径
MOTOR_TAB_R = 1.5      # モータマウントのネジまわりの耳の半径

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
CENTER_X = -13.4               # 機体の中心線
BOARD_ARC_R = 13.5             # 基板の前後のえぐれ（円弧）の半径
FRONT_ARC_CY = -43.32          # 前のえぐれの円の中心 Y
REAR_ARC_CY = 32.68            # 後ろのえぐれの円の中心 Y
EDGE_MARGIN = 0.8              # 基板の端から枠の外側までの距離
HALF_W = 9.3                   # 中心線から枠の外側までの距離（左右）
# HALF_W の決め方: 前のセンサホルダ（基板の下に 0.2mm 出ている）が中心線から 9.97mm、
# モータマウントのネジ頭が 10.4mm から始まるので、それより内側にする

# 枠を留めるネジ（どれも今付いているネジを外し、0.8mm 以上長いものに替えて留め直す）
POST_SCREWS = [(-22.3, -26.07), (-4.5, -26.07)]      # 支柱のネジ（前）
MOTOR_SCREWS_FRONT = [(-25.3, -9.07), (-1.5, -9.07)]  # モータマウントのネジ（前側）
MOTOR_SCREWS_REAR = [(-25.3, 8.43), (-1.5, 8.43)]     # モータマウントのネジ（後ろ側）

SLIDER_SIDE_U = 7.0            # スライダーの出っ張りの左右位置（中心線からの距離）

# スカートなしで使うスライダーを枠から切り出す位置
FRONT_SLIDER_END_Y = -22.5     # 前スライダーはこれより前
REAR_SLIDER_START_Y = 4.5      # 後ろスライダーはこれより後ろ


def outline(inset):
    """枠の外形（上から見た形）。inset だけ内側に縮めた形を返す。

    前後は基板のえぐれと同じ中心の円弧、左右はまっすぐな線。
    inset = 0 で枠の外側の面、inset = WALL_T で内側の面になる。
    """
    arc_r = BOARD_ARC_R + EDGE_MARGIN + inset
    half_w = HALF_W - inset
    # 円弧と左右の線が交わる点の Y
    dy = math.sqrt(arc_r * arc_r - half_w * half_w)
    front_corner_y = FRONT_ARC_CY + dy
    front_middle_y = FRONT_ARC_CY + arc_r
    rear_corner_y = REAR_ARC_CY - dy
    rear_middle_y = REAR_ARC_CY - arc_r
    left_x = CENTER_X + half_w
    right_x = CENTER_X - half_w
    return (cq.Workplane("XY")
            .moveTo(right_x, front_corner_y)
            .threePointArc((CENTER_X, front_middle_y), (left_x, front_corner_y))
            .lineTo(left_x, rear_corner_y)
            .threePointArc((CENTER_X, rear_middle_y), (right_x, rear_corner_y))
            .close())


def extrude_down(sketch, height):
    """z = 0（基板下面）から下に height だけ押し出す。"""
    return sketch.extrude(height).translate((0, 0, -height))


def cylinder_down(x, y, radius, height):
    """z = 0 から下に伸びる円柱。"""
    return extrude_down(cq.Workplane("XY").center(x, y).circle(radius), height)


def wall():
    """スカートの壁（輪の形）。"""
    outside = extrude_down(outline(0), FRAME_H)
    inside = extrude_down(outline(WALL_T), FRAME_H + 1)
    return outside.cut(inside)


def cut_post_screw(body, x, y):
    """支柱のネジ穴をあける。下から座ぐりでネジ頭を沈め、上 SCREW_PLATE_T だけ残す。"""
    head = cylinder_down(x, y, SCREW_HEAD_D / 2, FRAME_H + 1).translate((0, 0, -SCREW_PLATE_T))
    hole = cylinder_down(x, y, SCREW_HOLE_D / 2, FRAME_H + 1).translate((0, 0, 0.5))
    return body.cut(head).cut(hole)


def motor_tab(x, y):
    """壁からモータマウントのネジまで伸ばす耳。ネジ頭は耳の下に出る。"""
    # 壁の厚みの中ほどから、ネジの位置までの板
    if x < CENTER_X:
        wall_x = CENTER_X - HALF_W + WALL_T / 2
    else:
        wall_x = CENTER_X + HALF_W - WALL_T / 2
    length = abs(x - wall_x)
    middle_x = (x + wall_x) / 2
    plate = extrude_down(cq.Workplane("XY").center(middle_x, y)
                         .rect(length, MOTOR_TAB_R * 2), SCREW_PLATE_T)
    tab = plate.union(cylinder_down(x, y, MOTOR_TAB_R, SCREW_PLATE_T))
    hole = cylinder_down(x, y, SCREW_HOLE_D / 2, SCREW_PLATE_T + 1).translate((0, 0, 0.5))
    return tab.cut(hole)


def slider_pad(x, y):
    """床に近づく出っ張り（スライダー）。下端の角を丸める。"""
    height = -FLOOR_Z - SLIDER_GAP
    pad = cylinder_down(x, y, SLIDER_R, height)
    return pad.faces("<Z").edges().fillet(SLIDER_EDGE_R)


def slider_positions():
    """スライダーの出っ張りの位置（前 2 か所、後ろ 2 か所）。

    基板のえぐれの円弧から 0.5mm 内側に出っ張りの端が来るようにする。
    """
    arc_r = BOARD_ARC_R + 0.5 + SLIDER_R
    dy = math.sqrt(arc_r * arc_r - SLIDER_SIDE_U * SLIDER_SIDE_U)
    front_y = FRONT_ARC_CY + dy
    rear_y = REAR_ARC_CY - dy
    right_x = CENTER_X - SLIDER_SIDE_U
    left_x = CENTER_X + SLIDER_SIDE_U
    return [(right_x, front_y), (left_x, front_y), (right_x, rear_y), (left_x, rear_y)]


def skirt_frame():
    """スカート枠一式（壁、ネジの耳、スライダーの出っ張り）。"""
    body = wall()
    for x, y in slider_positions():
        body = body.union(slider_pad(x, y))
    for x, y in MOTOR_SCREWS_FRONT + MOTOR_SCREWS_REAR:
        body = body.union(motor_tab(x, y))
    # 支柱のネジは壁の上に来るので、壁を太くしてから穴をあける
    for x, y in POST_SCREWS:
        body = body.union(cylinder_down(x, y, POST_BOSS_R, FRAME_H))
        body = cut_post_screw(body, x, y)
    return body


def keep_y_range(body, y_min, y_max):
    """y_min〜y_max の範囲だけを残す。"""
    box = cq.Workplane("XY").box(100, y_max - y_min, 20).translate(
        (CENTER_X, (y_min + y_max) / 2, 0))
    return body.intersect(box)


def outline_length():
    """枠の外側の周の長さ（フィルムやテープを巻く長さの目安）。"""
    return outline(0).wires().val().Length()


def export(body, name):
    """STEP は機体の座標のまま、STL は基板側の面を下にして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    flipped = body.rotate((0, 0, 0), (1, 0, 0), 180)
    bb = flipped.val().BoundingBox()
    flipped = flipped.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(flipped, str(OUT / f"{name}.stl"),
                        tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    frame = skirt_frame()
    export(frame, "skirt_frame")
    export(keep_y_range(frame, -50, FRONT_SLIDER_END_Y), "slider_front")
    export(keep_y_range(frame, REAR_SLIDER_START_Y, 50), "slider_rear")
    print(f"枠の外周の長さ: {outline_length():.1f} mm")
    print(f"テープを床から 0.3mm まで垂らすときの幅: {-FLOOR_Z - 0.3:.2f} mm")
    print("done")
