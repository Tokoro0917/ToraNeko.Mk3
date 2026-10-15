"""エンコーダ基板はんだ付け治具（左右）とスロット幅テスト片を生成する。

使い方:  python3 encoder_solder_jig.py   （cadquery が必要: pip install cadquery）
出力:    同じディレクトリに *.stl / *.step

座標は実機の基板を部品面（F面）から見た向き。KiCad 座標 (xk, yk) を
(x, y) = (xk, -yk) に変換し、z = 0 をメイン基板の上面とする。

【治具の留め方】
ネジは使わず、メイン基板の縁を挟んで留める。
治具の外側の面は、ちょうどタイヤ用の切り欠きの縁（基板の端）と同じ位置にある。
そこに、基板の縁を上下から挟む「コの字のつめ」を付けてある。
  - つめの奥の面が基板の縁に当たって、チップ・磁石の方向（左右）の位置が決まる
  - つめの後ろの端が、切り欠きの後ろの端に当たって、前後の位置が決まる
  - 基板の上面には治具の底面が、下面にはつめの下あごが当たる
付けるときは、つめを切り欠きの中に下ろしてから、横に滑らせて基板の縁にはめる。
"""
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent

# ---- 調整用パラメータ ------------------------------------------------------
ENC_T = 0.6          # エンコーダ基板の厚さ（発注した板厚に合わせる）
ENC_W = 6.0          # エンコーダ基板の幅
ENC_H = 8.5          # エンコーダ基板の高さ
SLOT_C = 0.0         # スロットの片側すきま。テスト片でいちばん小さい溝（0.60mm）がちょうど固定できたので 0
CHIP_T = 1.14        # AS5047P（TSSOP14）の基板面からの高さ（STEP 上の値）
CHIP_L = 5.1         # AS5047P の長さ（基板幅方向）
CHIP_C = 0.10        # チップ上面とポケットのすきま
TOP = 10.5           # 治具の高さ

# つめ（基板の縁を挟む部分）
BOARD_T = 0.6        # メイン基板の厚さ
CLAMP_C = 0.0        # 基板を挟む溝のすきま（合計）。エンコーダのスロットと同じく 0 でちょうど固定できる見込み
LEG_W = 2.4          # つめの縦の部分（切り欠きの中に入る部分）の幅
JAW_L = 3.0          # 下あごが基板の下に入り込む長さ
JAW_T = 1.2          # 下あごの厚さ（0.4mm ノズルで 3 周）
JAW_LEAD = 0.4       # 下あごの先の面取り（基板の下に滑り込ませやすくする）
STOP_C = 0.1         # つめの後ろの端と、切り欠きの後ろの端のすきま

# ---- 基板上の位置（KiCad 座標）-------------------------------------------
ENC_Y = 66.05        # 磁石の軸（TN_ENC_Base の中心）
SIDES = {
    # 名前: (TN_ENC_Base の x, チップが向く方向 ±1)
    "R": (118.55, +1),   # U6、機体右
    "L": (101.25, -1),   # U4、機体左
}
EDGE_U = 4.75                   # エンコーダ中心から、タイヤ用切り欠きの縁（基板の端）まで（チップ側）
CUTOUT_REAR_VK = 6.5            # 切り欠きの後ろの端（ENC_Y からの KiCad y 差。72.55 − 66.05）
CUTOUT_CORNER_R = 1.0           # 切り欠きの角の丸み
MOUNT_U = (1.75, EDGE_U)        # 土台の範囲（エンコーダ中心からチップ側への距離）。モータマウントの足元と同じ
BASE_VK = (-5.0, 15.5)          # 治具の土台の範囲（KiCad y 差）。-5.0 より前は壁センサ LED
TOWER_VK = (BASE_VK[0], 4.0)    # エンコーダを挟む部分（とつめ）の範囲

# ---- 断面形状（u: エンコーダ中心からチップ側へ, w: 基板上面から上）---------
S = ENC_T / 2 + SLOT_C          # スロット面の位置
BACK_T = 2.0                    # 裏側の壁の厚さ
BACK_W = 3.0                    # 裏側の壁の下端（裏面パッドにこてを入れるため空ける）
END_W = 1.4                     # スロット端・チップ側の下端
GAP_W = BOARD_T + CLAMP_C       # 基板を挟む溝の高さ（下あごの上面は w = -GAP_W）
JAW_BOTTOM = -(GAP_W + JAW_T)   # 下あごの下面
TOWER_PROFILE = [
    # 上の面から、外側（チップ側）へ
    (-S - BACK_T, TOP), (EDGE_U + LEG_W, TOP),
    # つめの縦の部分を下りて、下あごの下面へ
    (EDGE_U + LEG_W, JAW_BOTTOM), (EDGE_U - JAW_L, JAW_BOTTOM),
    # 下あごの先（上の角を面取り）
    (EDGE_U - JAW_L, -GAP_W - JAW_LEAD), (EDGE_U - JAW_L + JAW_LEAD, -GAP_W),
    # 下あごの上面 → 基板の縁に当たる面 → 治具の底面
    (EDGE_U, -GAP_W), (EDGE_U, 0), (MOUNT_U[0], 0),
    # 下面の 45° 面はこての逃げ
    (S, END_W), (-S, END_W), (-S, BACK_W), (-S - BACK_T, BACK_W + BACK_T),
]
BASE_H = 4.0


def to_xy(side, u, vk):
    x0, d = SIDES[side]
    return x0 + d * u, -(ENC_Y + vk)


def prism(side, profile, vk0, vk1):
    """(u, w) 断面を vk0..vk1 の範囲で押し出した立体。"""
    x0, d = SIDES[side]
    pts = [(x0 + d * u, w) for u, w in profile]
    y0, y1 = sorted((-(ENC_Y + vk0), -(ENC_Y + vk1)))
    return (cq.Workplane("XZ", origin=(0, y1, 0)).polyline(pts).close()
            .extrude(y1 - y0))


def box(side, u0, u1, vk0, vk1, w0, w1):
    xa, ya = to_xy(side, u0, vk0)
    xb, yb = to_xy(side, u1, vk1)
    return (cq.Workplane("XY")
            .box(abs(xb - xa), abs(yb - ya), w1 - w0, centered=False)
            .translate((min(xa, xb), min(ya, yb), w0)))


def rear_stop(side):
    """つめの後ろの端を、切り欠きの後ろの端まで伸ばした止め。

    切り欠きの角は丸い（半径 1mm）ので、角を避けて、縁から 1.1mm より外側だけを伸ばす。
    断面はつめの縦の部分の内側なので、前の端面を下にして刷っても宙に浮かない。
    """
    u0 = EDGE_U + CUTOUT_CORNER_R + 0.1
    u1 = EDGE_U + LEG_W
    return box(side, u0, u1, TOWER_VK[1], CUTOUT_REAR_VK - STOP_C, JAW_BOTTOM, 0)


def jig(side):
    body = prism(side, TOWER_PROFILE, *TOWER_VK)
    body = body.union(box(side, MOUNT_U[0], MOUNT_U[1], *BASE_VK, 0, BASE_H))
    body = body.union(rear_stop(side))
    # エンコーダ基板のスロット（上端は基板の上端を受ける）
    body = body.cut(box(side, -S, S, -ENC_W / 2 - SLOT_C, ENC_W / 2 + SLOT_C, -1, ENC_H + SLOT_C))
    # AS5047P の逃げ（両端の細い帯と上端の帯で基板のチップ面を受ける）
    pocket_l = CHIP_L / 2 + 0.2
    body = body.cut(box(side, 0, ENC_T / 2 + CHIP_T + CHIP_C, -pocket_l, pocket_l, -1, 8.0))
    # 上面に L / R を彫る
    tx, ty = to_xy(side, 3.25, 9.0)
    label = (cq.Workplane("XY").workplane(offset=BASE_H - 0.4).center(tx, ty)
             .text(side, 3.0, 1.0, halign="center", valign="center"))
    body = body.cut(label)
    return body


def slot_test():
    """スロット幅の確認用。エンコーダ基板の端を差し込んで、軽く入って遊びのない幅を選ぶ。"""
    widths = [ENC_T + 2 * c for c in (0.0, 0.025, 0.05, 0.075, 0.10, 0.125)]
    pitch = 4.0
    blk = cq.Workplane("XY").box(pitch * len(widths) + 2, 10, 6, centered=False)
    for i, w in enumerate(widths):
        x = 1 + pitch * i + pitch / 2
        blk = blk.cut(cq.Workplane("XY").box(w, 6.4, 7, centered=(True, False, False))
                      .translate((x, 3.6, 1.5)))
        blk = blk.cut(cq.Workplane("XY").workplane(offset=5.6).center(x, 1.8)
                      .text(f"{w:.2f}"[1:], 1.6, 1.0, halign="center", valign="center"))
    return blk


def export(wp, name, origin, step=True):
    shape = wp.translate(origin)
    cq.exporters.export(shape, str(OUT / f"{name}.stl"), tolerance=0.01, angularTolerance=0.1)
    if step:
        cq.exporters.export(shape, str(OUT / f"{name}.step"))


if __name__ == "__main__":
    for side in SIDES:
        j = jig(side)
        bb = j.val().BoundingBox()
        # 出力は治具の左下を原点付近に寄せる（z はつめの下面を 0 にする）
        export(j, f"encoder_jig_{side}", (-bb.xmin, -bb.ymin, -bb.zmin))
    export(slot_test(), "slot_width_test", (0, 0, 0), step=False)
    print("done")
