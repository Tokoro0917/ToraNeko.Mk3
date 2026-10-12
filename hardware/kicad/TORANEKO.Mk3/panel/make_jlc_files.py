"""JLCPCB PCBA 用の BOM と CPL を、メイン基板パネル（TORANEKO.Mk3_main_panel.kicad_pcb）から作る。

PCBA で実装するのは下の PARTS に書いた部品だけ（抵抗・コンデンサ・LED などの安い部品と、手はんだが難しい IC）。
U2・コネクタ・THT 部品・在庫のない部品は手はんだ。
LCSC 番号は 2026-10 に JLCPCB の部品検索で在庫を確認したもの。
"""
import csv
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, 'TORANEKO.Mk3_main_panel.kicad_pcb')

# comment, footprint (JLC 表示用), LCSC, designators
PARTS = [
    ('100nF 16V X7R 0402', '0402', 'C1525',
     'C4 C7 C15 C16 C18 C19 C21 C22 C23 C24 C25 C28 C29 C30'),
    ('10uF 6.3V X5R 0402', '0402', 'C15525', 'C17 C20'),
    ('1uF 25V X5R 0402', '0402', 'C52923', 'C3 C6'),
    ('22uF 6.3V X5R 0603', '0603', 'C59461', 'C9 C10 C11 C12 C13 C14'),
    ('5.1R 1% 0402', '0402', 'C137955', 'R10 R11 R12 R13'),
    ('47k 1% 0402', '0402', 'C25792', 'R6'),
    ('10k 1% 0402', '0402', 'C25744', 'R2 R4 R8 R9 R22'),
    ('2k 1% 0402', '0402', 'C4109', 'R19 R20 R21 R27 R28 R29 R30'),
    ('1k 1% 0402', '0402', 'C11702', 'R23 R24 R25 R26'),
    ('100k 1% 0402', '0402', 'C25741', 'R5'),
    ('1.3k 1% 0402', '0402', 'C25866', 'R7'),
    ('LED Blue 0402', '0402', 'C130724', 'D1 D18 D19 D20 D21 D22 D23 D24'),
    ('AO3401A P-MOSFET', 'SOT-23', 'C15127', 'Q1'),
    ('AO3400A N-MOSFET', 'SOT-23', 'C20917', 'Q5'),
    ('STM32F411CEU6', 'UFQFPN-48', 'C60420', 'IC6'),
    ('LSM6DSRXTR', 'LGA-14', 'C2970339', 'U1'),
    ('MP6551GQB-Z', 'QFN-14 2.5x3', 'C3681210', 'IC1 IC2'),
    ('NCR321PASX', 'DFN2020-6', 'C3190647', 'IC3 IC4 IC8 IC9'),
]

s = open(PCB).read()
pos = {}
for m in re.finditer(r'\n\t\(footprint\s+"[^"]+"', s):
    head = s[m.start():m.start() + 4000]
    at = re.search(r'\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', head)
    ref = re.search(r'\(property "Reference" "([^"]+)"', head)
    if ref:
        pos[ref.group(1)] = (float(at.group(1)), float(at.group(2)), float(at.group(3) or 0))

refs_all = []
with open(os.path.join(HERE, 'BOM_JLC.csv'), 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['Comment', 'Designator', 'Footprint', 'LCSC Part #'])
    for comment, fp, lcsc, refs in PARTS:
        r = refs.split()
        missing = [x for x in r if x not in pos]
        assert not missing, f'not on the panel: {missing}'
        refs_all += r
        w.writerow([comment, ','.join(r), fp, lcsc])

# CPL: same convention as KiCad's position file (mm, Y up = -Y of the board file)
with open(os.path.join(HERE, 'CPL_JLC.csv'), 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation'])
    for r in sorted(refs_all, key=lambda x: (re.sub(r'\d', '', x), int(re.sub(r'\D', '', x)))):
        x, y, rot = pos[r]
        w.writerow([r, f'{x:.4f}', f'{-y:.4f}', 'Top', f'{rot % 360:.1f}'])

print('BOM lines', len(PARTS), 'placements', len(refs_all))
