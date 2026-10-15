"""メンテナンス用スタンド（機体を載せるとタイヤが浮く台）を生成する。

使い方:  python3 maintenance_stand.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに maintenance_stand.stl / .step

機体を 4 本の柱で基板の裏から支え、タイヤを机から浮かせる。
モータやエンコーダの確認（ファームウェアの動作確認）を、机の上で機体が走り出さずにできる。
柱はスカート枠・スライダーの外側に当たるので、どちらを付けたままでも載せられる。
四隅の L 字のガイドで、載せた機体が前後左右にずれないようにする。

座標は機体の STEP（mechanical/exported/TORANEKO.Mk3_v36.step）と同じ。
  z = 0 がメイン基板の下面、前が -Y、機体の右が -X、機体の中心線は X = -13.4
STEP はこの座標のまま（機体を載せた状態の位置で）出力する。
STL はそのまま印刷できる向き（底面を z = 0 にずらしただけ）で出力する。
"""
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 調整用パラメータ ------------------------------------------------------
TIRE_LIFT = 3.0        # タイヤの下端と台の上面のすきま
BASE_T = 2.0           # 台の板の厚さ
POST_D = 4.0           # 柱の太さ
GUIDE_T = 1.2          # 四隅のガイドの厚さ
GUIDE_GAP = 0.4        # 基板の端とガイドのすきま
GUIDE_LEN = 8.0        # ガイドの長さ（角から）
GUIDE_TOP_Z = 1.0      # ガイドの上端（基板の下面から。基板の上面は 0.6）
BASE_MARGIN = 2.0      # 基板の外形から台の板の端まで

# ---- 機体の寸法（STEP 座標）-----------------------------------------------
FLOOR_Z = -2.36                # タイヤの下端（基板の下面から）
CENTER_X = -13.4               # 機体の中心線
BOARD_X0 = -33.4               # 基板の外形（右端）
BOARD_X1 = 6.6                 # 基板の外形（左端）
BOARD_Y0 = -37.32              # 基板の外形（前端）
BOARD_Y1 = 26.68               # 基板の外形（後端）
BOARD_CORNER_R = 4.0           # 基板の四隅の丸み
# 柱が当たる位置。基板の裏に何も出ていない場所（センサのリード、コネクタのピン、
# スカート枠、スライダー、タイヤから 1mm 以上離れている）
POSTS = [(CENTER_X - 15.5, -22.0), (CENTER_X + 15.5, -22.0),
         (CENTER_X - 15.5, 20.0), (CENTER_X + 15.5, 20.0)]

# ---- 計算で決まる高さ ------------------------------------------------------
BASE_TOP_Z = FLOOR_Z - TIRE_LIFT
BASE_BOTTOM_Z = BASE_TOP_Z - BASE_T


def rounded_rect(margin, z0, z1):
    """基板の外形（四隅が丸い長方形）を margin だけ広げた形を、z0〜z1 で押し出す。"""
    width = BOARD_X1 - BOARD_X0 + margin * 2
    length = BOARD_Y1 - BOARD_Y0 + margin * 2
    center_y = (BOARD_Y0 + BOARD_Y1) / 2
    center_x = (BOARD_X0 + BOARD_X1) / 2
    radius = BOARD_CORNER_R + margin
    return (cq.Workplane("XY").workplane(offset=z0)
            .center(center_x, center_y)
            .rect(width, length).extrude(z1 - z0)
            .edges("|Z").fillet(radius))


def base_plate():
    """台の板。"""
    return rounded_rect(BASE_MARGIN, BASE_BOTTOM_Z, BASE_TOP_Z)


def posts():
    """基板の裏を支える柱 4 本。"""
    result = None
    for x, y in POSTS:
        post = (cq.Workplane("XY").workplane(offset=BASE_TOP_Z - 0.1)
                .center(x, y).circle(POST_D / 2).extrude(-BASE_TOP_Z + 0.1))
        if result is None:
            result = post
        else:
            result = result.union(post)
    return result


def corner_guides():
    """四隅の L 字のガイド。基板の外形に沿った枠を作り、四隅だけ残す。"""
    outer = rounded_rect(GUIDE_GAP + GUIDE_T, BASE_TOP_Z - 0.1, GUIDE_TOP_Z)
    inner = rounded_rect(GUIDE_GAP, BASE_TOP_Z - 1, GUIDE_TOP_Z + 1)
    ring = outer.cut(inner)
    corners = None
    for x in (BOARD_X0, BOARD_X1):
        for y in (BOARD_Y0, BOARD_Y1):
            box = (cq.Workplane("XY")
                   .box(GUIDE_LEN * 2, GUIDE_LEN * 2, 30)
                   .translate((x, y, 0)))
            if corners is None:
                corners = box
            else:
                corners = corners.union(box)
    return ring.intersect(corners)


def front_mark():
    """台の上面の前側に「FRONT」と彫る。"""
    return (cq.Workplane("XY").workplane(offset=BASE_TOP_Z - 0.6)
            .center(CENTER_X, BOARD_Y0 + 4.0)
            .text("FRONT", 4.0, 1.0, halign="center", valign="center"))


def stand():
    """スタンド一式。"""
    body = base_plate().union(posts()).union(corner_guides())
    return body.cut(front_mark())


def export(body, name):
    """STEP は機体の座標のまま、STL は底面を z = 0 にずらして出力する。"""
    cq.exporters.export(body, str(OUT / f"{name}.step"))
    bb = body.val().BoundingBox()
    moved = body.translate((-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(moved, str(OUT / f"{name}.stl"), tolerance=0.02, angularTolerance=0.2)


if __name__ == "__main__":
    export(stand(), "maintenance_stand")
    print(f"柱の高さ（台の上面から）: {-BASE_TOP_Z:.2f} mm")
    print("done")
