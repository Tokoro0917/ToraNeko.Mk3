"""お弁当箱ケースの中子（中の小さい箱）に入れて、機体を動かないように納める受け台を生成する。

使い方:  python3 case_insert.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに case_insert.stl / .step

中子の底に置く板。板の外形が中子の底に合っていて、中子の中で動かない。
機体は 4 本の柱で基板の裏を支え（タイヤは浮く）、四隅の L 字のガイドで前後左右にずれないようにする。
スカート枠・スライダーはどこにも触れないので、運ぶあいだに変形しない。
板の真ん中（スカートの下）は抜いて、材料を減らしてある。

【中子の寸法は写真からの推定】
使っているお弁当箱（学研ステイフル 11ぴきのねこ ランチボックス K14503）の中子に機体を入れた写真を、
基板（40 × 64mm）を物差しにして測った値（誤差 ±3〜5mm）。
  - 底は約 65 × 82mm の D 字型。機体の左側（前を上にして見たとき）の 2 つの角が大きく丸く（約 R20）、右側の 2 つは小さい（約 R5）
  - 誤差を見込んで、少し小さめ（64 × 80mm、左の角 R22、右の角 R6）にしてある
  - 深さは写真では測れないので、前の推定（27mm）のまま
実物を測ったら、下の「中子の寸法」のパラメータを書き換えて作り直す。

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面、前が -Y、機体の右が -X、機体の中心は (X, Y) = (-13.4, -5.32)
STEP はこの座標のまま（機体を載せた状態の位置で）出力する。
STL はそのまま印刷できる向き（底面を z = 0 にずらしただけ）で出力する。
"""
import math
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 中子の寸法（仮の値。実物を測って変える）-----------------------------
BOX_BOTTOM_L = 80.0    # 中子の内側の底の長さ（長いほう。機体の前後の向き）
BOX_BOTTOM_W = 64.0    # 中子の内側の底の幅（短いほう。機体の左右の向き）
BOX_CORNER_R_LEFT = 22.0   # 中子の内側の角の丸み（底のところ）。機体の左側（丸い側）の 2 つ
BOX_CORNER_R_RIGHT = 6.0   # 機体の右側（まっすぐな側）の 2 つ
BOX_DEPTH = 27.0       # 中子の内側の深さ（写真では測れないので前の推定のまま）

# ---- 調整用パラメータ ------------------------------------------------------
BOX_GAP = 0.5          # 板と中子の壁のすきま（片側）
PLATE_T = 2.0          # 板の厚さ
TIRE_LIFT = 1.5        # タイヤの下端と板の上面のすきま
POST_D = 4.0           # 柱の太さ
GUIDE_T = 1.6          # 四隅のガイドの厚さ
GUIDE_GAP = 0.4        # 基板の端とガイドのすきま
GUIDE_LEN = 8.0        # ガイドの長さ（角から）
GUIDE_ABOVE = 0.4      # ガイドの上端を、基板の上面からどれだけ出すか
WINDOW_HALF_W = 11.0   # 板の真ん中を抜く範囲（左右の半分）。スカート枠（中心から 9.3mm）より外まで
WINDOW_FRONT_V = -30.0 # 板の真ん中を抜く範囲の前の端（機体の中心から。マイナスが前）。スカート枠の前端（-22.3）より前
WINDOW_REAR_V = 20.0   # 板の真ん中を抜く範囲の後ろの端
WINDOW_R = 4.0         # 抜く範囲の角の丸み

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
FLOOR_Z = -2.36                # タイヤの下端（基板の下面から）
ROBOT_CENTER = (-13.4, -5.32)  # 機体の中心（基板の外形の中心 = 車軸の中点）
BOARD_W = 40.0                 # 基板の幅
BOARD_L = 64.0                 # 基板の長さ
BOARD_T = 0.6                  # 基板の厚さ
BOARD_CORNER_R = 4.0           # 基板の四隅の丸み
ROBOT_TOP_Z = 18.1             # 機体のいちばん高いところ（吸引ファンモータの上端。基板の下面から）
# 柱が当たる位置（機体の中心から、左 u・後ろ v）。メンテナンス用スタンドと同じ場所で、
# 基板の裏に何も出ていないところ（センサのリード、コネクタのピン、スカート枠、スライダー、タイヤから 1mm 以上離れている）
POSTS = [(-15.5, -16.68), (15.5, -16.68), (-15.5, 25.32), (15.5, 25.32)]

# ---- 計算で決まる高さ（STEP の z）-----------------------------------------
POST_H = -FLOOR_Z + TIRE_LIFT                 # 板の上面から基板の下面まで
PLATE_TOP_Z = -POST_H
PLATE_BOTTOM_Z = PLATE_TOP_Z - PLATE_T        # = 中子の底
GUIDE_TOP_Z = BOARD_T + GUIDE_ABOVE


def at(u, v, z):
    """機体の中心から左 u、後ろ v の位置を STEP 座標にする（u は左がプラス、v は後ろがプラス）。"""
    return (ROBOT_CENTER[0] + u, ROBOT_CENTER[1] + v, z)


def rounded_rect(width, length, radius, z0, z1):
    """機体の中心を真ん中にした、角の丸い長方形（幅 width は左右、長さ length は前後）を z0〜z1 で作る。"""
    center = at(0, 0, z0)
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(center[0], center[1])
            .rect(width, length).extrude(z1 - z0)
            .edges("|Z").fillet(radius))


def d_shape(width, length, r_left, r_right, z0, z1):
    """左（+X）の 2 つの角を r_left、右（-X）の 2 つを r_right で丸めた長方形（中子の底の D 字型）。"""
    center = at(0, 0, z0)
    body = (cq.Workplane("XY").workplane(offset=z0)
            .center(center[0], center[1])
            .rect(width, length).extrude(z1 - z0))
    body = body.edges("|Z").edges(f">X").fillet(r_left)
    return body.edges("|Z").edges(f"<X").fillet(r_right)


def plate():
    """中子の底に合わせた板。真ん中（スカートの下）を抜く。"""
    width = BOX_BOTTOM_W - BOX_GAP * 2
    length = BOX_BOTTOM_L - BOX_GAP * 2
    body = d_shape(width, length, BOX_CORNER_R_LEFT - BOX_GAP, BOX_CORNER_R_RIGHT - BOX_GAP,
                   PLATE_BOTTOM_Z, PLATE_TOP_Z)
    # 真ん中を抜く範囲（機体の座標でまっすぐな角の丸い長方形）
    window_center = at(0, (WINDOW_FRONT_V + WINDOW_REAR_V) / 2, PLATE_BOTTOM_Z - 1)
    window = (cq.Workplane("XY").workplane(offset=PLATE_BOTTOM_Z - 1)
              .center(window_center[0], window_center[1])
              .rect(WINDOW_HALF_W * 2, WINDOW_REAR_V - WINDOW_FRONT_V).extrude(PLATE_T + 2)
              .edges("|Z").fillet(WINDOW_R))
    return body.cut(window)


def posts():
    """基板の裏を支える柱 4 本。"""
    result = None
    for u, v in POSTS:
        base = at(u, v, PLATE_TOP_Z - 0.1)
        post = (cq.Workplane("XY").workplane(offset=base[2])
                .center(base[0], base[1]).circle(POST_D / 2).extrude(POST_H + 0.1))
        if result is None:
            result = post
        else:
            result = result.union(post)
    return result


def corner_guides():
    """基板の四隅の L 字のガイド。基板の外形に沿った枠を作り、四隅だけ残す。"""
    outer = rounded_rect(BOARD_W + (GUIDE_GAP + GUIDE_T) * 2, BOARD_L + (GUIDE_GAP + GUIDE_T) * 2,
                         BOARD_CORNER_R + GUIDE_GAP + GUIDE_T, PLATE_TOP_Z - 0.1, GUIDE_TOP_Z)
    inner = rounded_rect(BOARD_W + GUIDE_GAP * 2, BOARD_L + GUIDE_GAP * 2,
                         BOARD_CORNER_R + GUIDE_GAP, PLATE_TOP_Z - 1, GUIDE_TOP_Z + 1)
    ring = outer.cut(inner)
    corners = None
    for u in (-BOARD_W / 2, BOARD_W / 2):
        for v in (-BOARD_L / 2, BOARD_L / 2):
            box = cq.Workplane("XY").box(GUIDE_LEN * 2, GUIDE_LEN * 2, 60).translate(at(u, v, 0))
            if corners is None:
                corners = box
            else:
                corners = corners.union(box)
    return ring.intersect(corners)


def front_mark():
    """板の上面の前側に「FRONT」と彫る。文字は 180° 回して、読める向きが機体の前になるようにする。"""
    position = at(0, WINDOW_FRONT_V - 4.5, 0)
    letters = (cq.Workplane("XY").workplane(offset=PLATE_TOP_Z - 0.4)
               .center(position[0], position[1])
               .text("FRONT", 4.0, 1.0, halign="center", valign="center"))
    return letters.rotate((position[0], position[1], 0), (position[0], position[1], 1), 180)


def case_insert():
    """受け台一式。"""
    body = plate().union(posts()).union(corner_guides())
    return body.cut(front_mark())


def print_fit():
    """機体と中子の壁・深さの余裕を表示する（中子の寸法を変えたときに確かめる用）。"""
    side_gap = BOX_BOTTOM_W / 2 - (BOARD_W / 2 + GUIDE_GAP + GUIDE_T)
    end_gap = BOX_BOTTOM_L / 2 - (BOARD_L / 2 + GUIDE_GAP + GUIDE_T)
    # 角: 中子の角の丸みの中心から、ガイドの角までの距離が、丸みの半径より小さければ入る（丸いほうの角で確かめる）
    radius = BOX_CORNER_R_LEFT
    corner_center_u = BOX_BOTTOM_W / 2 - radius
    corner_center_v = BOX_BOTTOM_L / 2 - radius
    guide_u = BOARD_W / 2 + GUIDE_GAP + GUIDE_T
    guide_v = BOARD_L / 2 + GUIDE_GAP + GUIDE_T
    corner_gap = radius - math.hypot(guide_u - corner_center_u, guide_v - corner_center_v)
    top_gap = BOX_DEPTH - (ROBOT_TOP_Z - PLATE_BOTTOM_Z)
    print(f"左右の余裕（片側）: {side_gap:.1f} mm")
    print(f"前後の余裕（片側）: {end_gap:.1f} mm")
    print(f"角の余裕: {corner_gap:.1f} mm")
    print(f"機体のいちばん上から中子の上の縁まで: {top_gap:.1f} mm（マイナスなら中子からはみ出す）")


def export(body, name):
    """STEP は機体の座標のまま、STL は底面を z = 0 にずらして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    bb = body.val().BoundingBox()
    moved = body.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(moved, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    export(case_insert(), "case_insert")
    print_fit()
    print("done")
