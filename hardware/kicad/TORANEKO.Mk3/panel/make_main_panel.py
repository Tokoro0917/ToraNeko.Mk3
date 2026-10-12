"""メイン基板の PCBA 用パネル（1 枚 + レール + フィデューシャル）を作る。

JLCPCB の Standard PCBA（0.6mm 厚）は 70x70mm 以上・レール・フィデューシャルが必要なため、
メイン基板の周りに枠を付ける。タブは左右のまっすぐな辺だけに付ける。
"""
import math
import os
import re
import uuid

from shapely.geometry import LineString, Point, box
from shapely.ops import polygonize, unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'TORANEKO.Mk3.kicad_pcb')
OUT = os.path.join(HERE, 'TORANEKO.Mk3_main_panel.kicad_pcb')

GAP = 2.0          # board to rail
RAIL_X = 13.0      # left/right rail width (panel width = 70mm)
RAIL_Y = 5.0       # top/bottom rail width
TABW = 2.5         # tab width
PX0, PY0 = 100.0, 100.0
EDGE_LAYERS_DROP = ('Eco1.User', 'Eco2.User', 'Dwgs.User', 'Cmts.User')

s = open(SRC).read()


def top_blocks(src):
    """top-level children of (kicad_pcb ...), aware of quoted strings"""
    i = src.index('(kicad_pcb') + 1
    d, st, inq, j = 0, None, False, i
    while j < len(src):
        c = src[j]
        if c == '"' and src[j - 1] != '\\':
            inq = not inq
        elif not inq:
            if c == '(':
                if d == 0:
                    st = j
                d += 1
            elif c == ')':
                d -= 1
                if d == 0 and st is not None:
                    yield src[st:j + 1]
                    st = None
                if d < 0:
                    break
        j += 1


tops = list(top_blocks(s))
kind = lambda b: re.match(r'\((\w+)', b).group(1)
layer = lambda b: (re.search(r'\(layer "([^"]+)"\)', b) or [None, None])[1]

# ---- board outline (Edge.Cuts) ----
def edge_geom(b):
    P = lambda k: tuple(map(float, re.search(r'\(' + k + r' ([-\d.]+) ([-\d.]+)\)', b).groups()))
    t = kind(b)
    if t == 'gr_line':
        return LineString([P('start'), P('end')])
    if t == 'gr_arc':
        (ax, ay), (bx, by), (cx, cy) = P('start'), P('mid'), P('end')
        D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / D
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / D
        r = math.hypot(ax - ux, ay - uy)
        t0, t1, t2 = (math.atan2(y - uy, x - ux) for x, y in ((ax, ay), (bx, by), (cx, cy)))
        n = lambda v: v % (2 * math.pi)
        span = n(t2 - t0) if n(t1 - t0) < n(t2 - t0) else n(t2 - t0) - 2 * math.pi
        k = max(8, int(abs(span) * r / 0.05))          # ~50um chords, sagitta < 1um
        return LineString([(ux + r * math.cos(t0 + span * i / k), uy + r * math.sin(t0 + span * i / k)) for i in range(k + 1)])
    if t == 'gr_circle':
        c, e = P('center'), P('end')
        return Point(c).buffer(math.dist(c, e), 256).exterior
    if t == 'gr_rect':
        a, c = P('start'), P('end')
        return box(min(a[0], c[0]), min(a[1], c[1]), max(a[0], c[0]), max(a[1], c[1])).exterior
    pts = [tuple(map(float, p)) for p in re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', b)]
    return LineString(pts + [pts[0]])


edges = [(b, edge_geom(b)) for b in tops if kind(b).startswith('gr_') and layer(b) == 'Edge.Cuts']
snap = lambda g: LineString([(round(x, 4), round(y, 4)) for x, y in g.coords])
board = sorted(polygonize(unary_union([snap(g) for _, g in edges])), key=lambda f: -f.area)[0]
BX0, BY0, BX1, BY1 = board.bounds
inside = lambda x, y: BX0 - 0.5 <= x <= BX1 + 0.5 and BY0 - 0.5 <= y <= BY1 + 0.5
outer_ring = board.exterior


def is_outer(g):
    return g.distance(outer_ring) < 1e-3 and outer_ring.buffer(1e-3).contains(g)


# ---- placement ----
PW = RAIL_X + GAP + (BX1 - BX0) + GAP + RAIL_X
PH = RAIL_Y + GAP + (BY1 - BY0) + GAP + RAIL_Y
DX = PX0 + RAIL_X + GAP - BX0
DY = PY0 + RAIL_Y + GAP - BY0

def shift_all(b):
    """shift every absolute coordinate token (blocks other than footprints)"""
    return re.sub(r'\((start|end|mid|center|xy|at) ([-\d.]+) ([-\d.]+)',
                  lambda m: f'({m.group(1)} {float(m.group(2)) + DX:.6f} {float(m.group(3)) + DY:.6f}', b)

def shift_fp(b):
    """footprint: only its own (at); children are relative"""
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)', b)
    return b[:m.start()] + f'(at {float(m.group(1)) + DX:.6f} {float(m.group(2)) + DY:.6f}' + b[m.end():]

def fp_xy(b):
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)', b)
    return float(m.group(1)), float(m.group(2))

def first_xy(b):
    m = re.search(r'\((?:start|at|center|xy) ([-\d.]+) ([-\d.]+)', b)
    return (float(m.group(1)), float(m.group(2))) if m else None

HEAD = ('version', 'generator', 'generator_version', 'general', 'paper', 'layers', 'setup', 'net')
head = [b for b in tops if kind(b) in HEAD]
out = []
skipped = {}
for b in tops:
    k = kind(b)
    if k in HEAD or k in ('group', 'generated', 'embedded_fonts'):
        continue
    if k == 'footprint':
        if inside(*fp_xy(b)):
            out.append(shift_fp(b))
        else:
            skipped[k] = skipped.get(k, 0) + 1          # encoder boards drawn beside the main board
        continue
    if k.startswith('gr_') and layer(b) == 'Edge.Cuts':
        g = dict(edges)[b]
        if not inside(*g.coords[0]):
            continue
        if not is_outer(g):
            out.append(shift_all(b))                  # inner cutouts stay exact
        continue
    if k.startswith('gr_') and layer(b) in EDGE_LAYERS_DROP:
        continue
    if k == 'zone':
        out.append(shift_all(b))
        continue
    xy = first_xy(b)
    if xy and not inside(*xy):
        skipped[k] = skipped.get(k, 0) + 1
        continue
    out.append(shift_all(b))

# ---- tabs: left/right straight edges only (not the wheel notches, not the curved ends) ----
# positions chosen where copper is >= 0.6mm and courtyards >= 1.8mm away from the edge
# (board coordinates; check again if parts near the side edges move)
TABS = [('L', 45.75), ('L', 85.5), ('R', 47.0), ('R', 74.75)]
import shapely.affinity as sa
boardP = sa.translate(board, DX, DY)
L, T, R, B = boardP.bounds
tabs = []
for side, yb in TABS:
    y = yb + DY
    tabs.append(box(PX0 + RAIL_X, y, L + 0.05, y + TABW) if side == 'L' else box(R - 0.05, y, PX0 + PW - RAIL_X, y + TABW))

inner_win = box(PX0 + RAIL_X, PY0 + RAIL_Y, PX0 + PW - RAIL_X, PY0 + PH - RAIL_Y)
cut = inner_win.difference(unary_union([boardP] + tabs))


def gr_poly(poly):
    pts = ' '.join(f'(xy {x:.4f} {y:.4f})' for x, y in list(poly.exterior.coords)[:-1])
    return f'(gr_poly (pts {pts}) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))'


polys = [cut] if cut.geom_type == 'Polygon' else list(cut.geoms)
for p in polys:
    out.append(gr_poly(p))
out.append(f'(gr_rect (start {PX0} {PY0}) (end {PX0 + PW:.4f} {PY0 + PH:.4f}) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))')

# mouse bites: NPTH 0.5mm, pitch 0.75, just outside the board edge
holes = []
for t in tabs:
    x0, y0, x1, y1 = t.bounds
    xc = L - 0.25 if x1 <= L + 0.1 else R + 0.25
    n = int((TABW - 0.5) / 0.75) + 1
    off = (TABW - 0.75 * (n - 1)) / 2
    holes += [(xc, y0 + off + 0.75 * k) for k in range(n)]
# tooling holes (NPTH 1.152) in the rail corners, fiducials (1mm Cu, 2mm mask) on the left/right rails
tooling = [(PX0 + 3.5, PY0 + 2.5), (PX0 + PW - 3.5, PY0 + 2.5), (PX0 + 3.5, PY0 + PH - 2.5), (PX0 + PW - 3.5, PY0 + PH - 2.5)]
fids = [(PX0 + RAIL_X / 2, PY0 + RAIL_Y + 3.0), (PX0 + RAIL_X / 2, PY0 + PH - RAIL_Y - 3.0), (PX0 + PW - RAIL_X / 2, PY0 + RAIL_Y + 3.0)]

def fp_block(name, ref, pads):
    lines = [f'(footprint "TORANEKO_Panel:{name}" (layer "F.Cu") (uuid "{uuid.uuid4()}") (at 0 0)',
             f'(property "Reference" "{ref}" (at {PX0 + 2} {PY0 + PH + 2} 0) (layer "F.Fab") (hide yes) (uuid "{uuid.uuid4()}") (effects (font (size 1 1) (thickness 0.15))))',
             f'(property "Value" "{name}" (at {PX0 + 2} {PY0 + PH + 3.5} 0) (layer "F.Fab") (hide yes) (uuid "{uuid.uuid4()}") (effects (font (size 1 1) (thickness 0.15))))',
             '(attr board_only exclude_from_pos_files exclude_from_bom)']
    lines += pads
    lines.append(')')
    return '\n'.join(lines)

out.append(fp_block('MouseBites', 'MB1', [f'(pad "" np_thru_hole circle (at {x:.4f} {y:.4f}) (size 0.5 0.5) (drill 0.5) (layers "*.Cu" "*.Mask") (uuid "{uuid.uuid4()}"))' for x, y in holes]))
out.append(fp_block('ToolingHoles', 'TH1', [f'(pad "" np_thru_hole circle (at {x:.4f} {y:.4f}) (size 1.152 1.152) (drill 1.152) (layers "*.Cu" "*.Mask") (uuid "{uuid.uuid4()}"))' for x, y in tooling]))
for i, (x, y) in enumerate(fids, 1):
    out.append(f'(footprint "TORANEKO_Panel:Fiducial_1mm" (layer "F.Cu") (uuid "{uuid.uuid4()}") (at {x:.4f} {y:.4f})\n'
               f'(property "Reference" "FID{i}" (at 0 -2 0) (layer "F.Fab") (hide yes) (uuid "{uuid.uuid4()}") (effects (font (size 0.8 0.8) (thickness 0.12))))\n'
               f'(property "Value" "Fiducial" (at 0 2 0) (layer "F.Fab") (hide yes) (uuid "{uuid.uuid4()}") (effects (font (size 0.8 0.8) (thickness 0.12))))\n'
               f'(attr smd board_only exclude_from_pos_files exclude_from_bom)\n'
               f'(pad "1" smd circle (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (solder_mask_margin 0.5) (uuid "{uuid.uuid4()}"))\n)')
out.append(f'(gr_text "TORANEKO.Mk3 main 0.6mm" (at {PX0 + PW / 2:.2f} {PY0 + RAIL_Y / 2:.2f} 0) (layer "F.SilkS") (uuid "{uuid.uuid4()}") (effects (font (size 1.2 1.2) (thickness 0.18))))')

res = '(kicad_pcb\n\t' + '\n\t'.join(head + out) + '\n)\n'
open(OUT, 'w').write(res)
print(f'panel {PW:.2f} x {PH:.2f} mm; board {BX1 - BX0:.2f} x {BY1 - BY0:.2f}; footprints',
      sum(1 for o in out if o.startswith('(footprint')), 'segments', sum(1 for o in out if o.startswith('(segment')),
      'vias', sum(1 for o in out if o.startswith('(via')), 'zones', sum(1 for o in out if o.startswith('(zone')),
      'mouse bites', len(holes), 'skipped', skipped)
