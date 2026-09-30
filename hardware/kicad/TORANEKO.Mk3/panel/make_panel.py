import re, uuid
from shapely.geometry import box, Polygon
from shapely.ops import unary_union
import os
HERE=os.path.dirname(os.path.abspath(__file__))
SRC=os.path.join(HERE,'..','TORANEKO.Mk3.kicad_pcb')
OUT=os.path.join(HERE,'TORANEKO.Mk3_panel.kicad_pcb')
UPPER_DXF=os.path.join(HERE,'..','上部固定基板外形.dxf')
s=open(SRC).read()
def blocks_top(src):
    # yield top-level child blocks of (kicad_pcb ...)
    i=src.index('(kicad_pcb')+1; d=0; start=None
    for j in range(i,len(src)):
        c=src[j]
        if c=='(':
            if d==0: start=j
            d+=1
        elif c==')':
            d-=1
            if d==0 and start is not None: yield src[start:j+1]; start=None
            if d<0: break
top=list(blocks_top(s))
NETS={m.group(2):int(m.group(1)) for m in re.finditer(r'\n\t\(net (\d+) "([^"]*)"\)',s)}
NEXT=[max(NETS.values())+1]; EXTRA=[]
def netmap(b,suffix):
    # give each panel copy its own nets so DRC does not see copies as unconnected
    if not suffix: return b
    used={}
    def repl_named(m):
        n,name=int(m.group(1)),m.group(2)
        if not name or name.startswith('unconnected') : return m.group(0)
        nn=name+suffix
        if nn not in NETS: NETS[nn]=NEXT[0]; NEXT[0]+=1; EXTRA.append((NETS[nn],nn))
        used[n]=NETS[nn]; return f'(net {NETS[nn]} "{nn}")'
    b=re.sub(r'\(net (\d+) "([^"]*)"\)',repl_named,b)
    return b
INV=None
head=[b for b in top if re.match(r'\((version|generator|generator_version|general|paper|layers|setup|net) ',b) or re.match(r'\((general|layers|setup)\b',b)]
BOARDS={'A':dict(rect=(75.0,83.303,81.0,91.803),refs=['U3','IC5','C26']),
        'B':dict(rect=(82.55,83.35,88.55,91.85),refs=['U5','IC7','C27'])}
fps={}
for b in top:
    if b.startswith('(footprint'):
        r=re.search(r'\(property "Reference" "([^"]+)"',b).group(1)
        fps[r]=b
def inside(x,y,rect,m=0.01):
    x0,y0,x1,y1=rect; return x0-m<=x<=x1+m and y0-m<=y<=y1+m
segs=[b for b in top if b.startswith('(segment')]; vias=[b for b in top if b.startswith('(via')]
def newuuid(t): return re.sub(r'\(uuid "[^"]+"\)',lambda m:f'(uuid "{uuid.uuid4()}")',t)
def shift_fp(b,dx,dy,suffix):
    m=re.search(r'\(at ([-\d.]+) ([-\d.]+)((?: [-\d.]+)?)\)',b)
    b=b[:m.start()]+f'(at {float(m.group(1))+dx:.4f} {float(m.group(2))+dy:.4f}{m.group(3)})'+b[m.end():]
    if suffix: b=re.sub(r'(\(property "Reference" ")([^"]+)(")',lambda m:m.group(1)+m.group(2)+suffix+m.group(3),b,count=1)
    return newuuid(b)
def shift_xy(b,dx,dy,keys):
    for k in keys:
        b=re.sub(r'\('+k+r' ([-\d.]+) ([-\d.]+)\)',lambda m:f'({k} {float(m.group(1))+dx:.4f} {float(m.group(2))+dy:.4f})',b)
    return newuuid(b)
# panel geometry
W,H=6.0,8.5; GAP=2.0; RAIL=5.0; BAR=3.0; N=6; PX0,PY0=100.0,100.0
# upper fixing plate (mechanical only): read from DXF, arcs flattened to 2 um
import ezdxf
from ezdxf import path as dxfpath
upper_loops=[]; upper_circles=[]
for e in ezdxf.readfile(UPPER_DXF).modelspace():
    if e.dxftype()=='LWPOLYLINE':
        upper_loops.append([(v.x,v.y) for v in dxfpath.make_path(e).flattening(0.002)])
    elif e.dxftype()=='CIRCLE':
        upper_circles.append((e.dxf.center.x,e.dxf.center.y,e.dxf.radius))
upper_loops.sort(key=lambda l:-Polygon(l).area)   # [0] = outline, rest = inner cutouts
UX0d,UY0d,UX1d,UY1d=Polygon(upper_loops[0]).bounds
UW,UH=UX1d-UX0d,UY1d-UY0d
BX0=PX0+RAIL+GAP; BY0=PY0+RAIL+GAP
PW=RAIL+GAP+N*W+(N-1)*GAP+GAP+RAIL; PH=RAIL+GAP+H+GAP+BAR+GAP+UH+GAP+RAIL
BAR_Y0=BY0+H+GAP; UY0=BAR_Y0+BAR+GAP; UX0=PX0+RAIL+GAP
TAB_T,TAB_B=BY0+0.5,BY0+2.5      # side tabs 0.5..2.5 mm from the top (far) edge; solder edge (bottom) stays clean
out=[]
boards=[];order=['A','B']*3
for i,kind in enumerate(order):
    bx=BX0+i*(W+GAP); x0,y0,_,_=BOARDS[kind]['rect']; dx,dy=bx-x0,BY0-y0
    rect=BOARDS[kind]['rect']; suffix='' if i<2 else f'_{i//2+1}'
    for r in BOARDS[kind]['refs']: out.append(netmap(shift_fp(fps[r],dx,dy,suffix),suffix))
    id2name={v:k for k,v in NETS.items()}
    def netnum(b):
        n=int(re.search(r'\(net (\d+)\)',b).group(1))
        if not suffix: return b
        nn=id2name[n]+suffix
        if nn not in NETS: NETS[nn]=NEXT[0]; NEXT[0]+=1; EXTRA.append((NETS[nn],nn))
        return re.sub(r'\(net \d+\)',f'(net {NETS[nn]})',b)
    for b in segs:
        pts=[tuple(map(float,p)) for p in re.findall(r'\((?:start|end) ([-\d.]+) ([-\d.]+)\)',b)]
        if all(inside(x,y,rect) for x,y in pts): out.append(netnum(shift_xy(b,dx,dy,['start','end'])))
    for b in vias:
        x,y=map(float,re.search(r'\(at ([-\d.]+) ([-\d.]+)\)',b).groups())
        if inside(x,y,rect): out.append(netnum(shift_xy(b,dx,dy,['at'])))
    boards.append(box(bx,BY0,bx+W,BY0+H))
# tabs
tabs=[]; edges=[PX0+RAIL]+[b.bounds[2] for b in boards]
lefts=[b.bounds[0] for b in boards]+[PX0+PW-RAIL]
for xl,xr in zip(edges,lefts): tabs.append(box(xl,TAB_T,xr,TAB_B))
# upper plate: DXF (x right, y up) -> KiCad (y down), left-top of the bbox at (UX0, UY0)
def u2k(x,y): return (UX0+(x-UX0d), UY0+(UY1d-y))
upper=Polygon([u2k(*p) for p in upper_loops[0]])
UL,UT,UR,UB=upper.bounds
# tabs on the straight edges only: left edge x2 (to the left rail), short top/bottom edges (to the bar / bottom rail)
TABW=2.0; OV=0.05
utabs_left=[(UT+4.0-TABW/2),(UB-4.0-TABW/2)]            # y0 of each left tab
utab_x0=UL+(-3.5-UX0d)-TABW/2   # centre of the 7.4mm straight top/bottom edge (DXF x -7.25..0.13)
tabs.append(box(PX0+RAIL,utabs_left[0],UL+OV,utabs_left[0]+TABW))
tabs.append(box(PX0+RAIL,utabs_left[1],UL+OV,utabs_left[1]+TABW))
tabs.append(box(utab_x0,BAR_Y0+BAR,utab_x0+TABW,UT+OV))
tabs.append(box(utab_x0,UB-OV,utab_x0+TABW,PY0+PH-RAIL))
win1=box(PX0+RAIL,PY0+RAIL,PX0+PW-RAIL,BAR_Y0)
win2=box(PX0+RAIL,BAR_Y0+BAR,PX0+PW-RAIL,PY0+PH-RAIL)
cut=unary_union([win1,win2]).difference(unary_union(boards+tabs+[upper]))
def gr_poly(poly):
    pts=' '.join(f'(xy {x:.4f} {y:.4f})' for x,y in list(poly.exterior.coords)[:-1])
    return f'(gr_poly (pts {pts}) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))'
polys=[cut] if cut.geom_type=='Polygon' else list(cut.geoms)
for p in polys: out.append(gr_poly(p))
for l in upper_loops[1:]: out.append(gr_poly(Polygon([u2k(*q) for q in l])))
for cx,cy,r in upper_circles:
    if r>1.0:
        x,y=u2k(cx,cy)
        out.append(f'(gr_circle (center {x:.4f} {y:.4f}) (end {x+r:.4f} {y:.4f}) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))')
out.append(f'(gr_rect (start {PX0} {PY0}) (end {PX0+PW} {PY0+PH}) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts") (uuid "{uuid.uuid4()}"))')
# mouse bites: NPTH 0.5mm, pitch 0.75, just outside the board edge (on the tab side)
holes=[]
for i,b in enumerate(boards):
    x0,_,x1,_=b.bounds
    for xc in (x0-0.25,x1+0.25):
        for k in range(3): holes.append((xc,TAB_T+0.25+0.75*k))
for y0 in utabs_left:
    for k in range(3): holes.append((UL-0.25,y0+0.25+0.75*k))
for yc in (UT-0.25,UB+0.25):
    for k in range(3): holes.append((utab_x0+0.25+0.75*k,yc))
npth=[(*u2k(cx,cy),2*r) for cx,cy,r in upper_circles if r<=1.0]   # plate screw holes
fp=[f'(footprint "TORANEKO_Panel:MouseBites" (layer "F.Cu") (uuid "{uuid.uuid4()}") (at 0 0)',
    f'(property "Reference" "MB1" (at {PX0+2} {PY0+2} 0) (layer "F.Fab") (uuid "{uuid.uuid4()}") (effects (font (size 1 1) (thickness 0.15))))',
    f'(property "Value" "MouseBites" (at {PX0+2} {PY0+3.5} 0) (layer "F.Fab") (uuid "{uuid.uuid4()}") (effects (font (size 1 1) (thickness 0.15))))',
    '(attr board_only exclude_from_pos_files exclude_from_bom)']
for x,y in holes: fp.append(f'(pad "" np_thru_hole circle (at {x:.4f} {y:.4f}) (size 0.5 0.5) (drill 0.5) (layers "*.Cu" "*.Mask") (uuid "{uuid.uuid4()}"))')
fp.append(')'); out.append('\n'.join(fp))
fp=[f'(footprint "TORANEKO_Panel:UpperPlateHoles" (layer "F.Cu") (uuid "{uuid.uuid4()}") (at 0 0)',
    f'(property "Reference" "H101" (at {UL+1} {UT+1} 0) (layer "F.Fab") (uuid "{uuid.uuid4()}") (effects (font (size 1 1) (thickness 0.15))))',
    f'(property "Value" "UpperPlateHoles" (at {UL+1} {UT+2.5} 0) (layer "F.Fab") (uuid "{uuid.uuid4()}") (effects (font (size 1 1) (thickness 0.15))))',
    '(attr board_only exclude_from_pos_files exclude_from_bom)']
for x,y,d in npth: fp.append(f'(pad "" np_thru_hole circle (at {x:.4f} {y:.4f}) (size {d:.2f} {d:.2f}) (drill {d:.2f}) (layers "*.Cu" "*.Mask") (uuid "{uuid.uuid4()}"))')
fp.append(')'); out.append('\n'.join(fp))
out.append(f'(gr_text "TORANEKO.Mk3 encoder x6 + upper plate  0.6mm" (at {PX0+PW/2:.2f} {PY0+2.5} 0) (layer "F.SilkS") (uuid "{uuid.uuid4()}") (effects (font (size 1.2 1.2) (thickness 0.18))))')
head+= [f'(net {n} "{nm}")' for n,nm in EXTRA]
# keep header order: nets must follow setup; head already ends with nets
res='(kicad_pcb\n\t'+'\n\t'.join(head+out)+'\n)\n'
import os; os.makedirs(os.path.dirname(OUT),exist_ok=True); open(OUT,'w').write(res)
print('panel',PW,'x',round(PH,3),'upper',round(UW,3),'x',round(UH,3),'npth',len(npth),'mm; footprints',sum(1 for o in out if o.startswith('(footprint')),'segments',sum(1 for o in out if o.startswith('(segment')),'vias',sum(1 for o in out if o.startswith('(via')),'holes',len(holes),'cut polys',len(polys))
