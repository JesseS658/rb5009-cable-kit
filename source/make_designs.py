#!/usr/bin/env python3
"""Parametric cable-management set for a MikroTik RB5009UG+S+IN on a QIDI MAX 4.

Run:  /opt/printerctl/designs/.venv/bin/python make_designs.py   -> out/*.stl + out/parts.json
All parts print in the orientation they are generated in (flat face on z=0), no supports, PLA.

Router facts (MikroTik manual): 220 x 125 x 22 mm, passive cooling through the case (do not
enclose it), front panel left->right: reset, DC jack (5.5/2.0), SFP+, USB 3.0, ether1-4, ether5-8;
2-pin power terminal + ground on the left end; mounting tabs at both ends.
Port X positions are estimates (standard 15.88 mm ganged-jack pitch); every slot is wide enough
that a few mm of error only angles the cable slightly.
"""
import json
import os

import numpy as np
import trimesh
import manifold3d
from manifold3d import CrossSection as CS, JoinType, Manifold as M

manifold3d.set_circular_segments(64)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# ---------------------------------------------------------------- tolerances / standard sizes
FIT = 0.3            # sliding/snap clearance per side
ROUTER = (220.0, 125.0, 22.0)
ROUTER_CLEAR = 1.5   # per side: tolerant of unit-to-unit and manual dimension variation
SLOT_W = 7.6         # Cat5e 5.0 / Cat6 6.0-6.5 round, flat cables 7 mm on edge, DAC/USB/fiber
CABLE_TAG_D = 6.2    # tag inner diameter (fits 5.5-6.5 mm cables by flexing)
# router-local X of each front connector (0 = left end of the router)
PORT_PITCH = 15.88
_G2 = [220 - 20 - PORT_PITCH * (3 - i) for i in range(4)]        # ether5..8
_G1 = [x - 4 * PORT_PITCH - 5.0 for x in _G2]                    # ether1..4
FRONT_SLOTS = [("P", 25.0), ("F", 47.0), ("U", 64.0)] + \
              [(str(i + 1), x) for i, x in enumerate(_G1 + _G2)]


# ---------------------------------------------------------------- primitives
def box(x0, y0, z0, x1, y1, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def rrect(x0, y0, x1, y1, r):
    return CS.square([x1 - x0 - 2 * r, y1 - y0 - 2 * r]).translate([x0 + r, y0 + r]).offset(r, JoinType.Round)


def rslab(x0, y0, x1, y1, z0, z1, r):
    return rrect(x0, y0, x1, y1, r).extrude(z1 - z0).translate([0, 0, z0])


def cyl(x, y, z0, h, r1, r2=None):
    return M.cylinder(h, r1, r1 if r2 is None else r2).translate([x, y, z0])


def poly(pts):
    """CrossSection from a simple polygon in either winding order."""
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return CS([pts if area > 0 else pts[::-1]])


def prism_yz(pts, x0, x1):
    """2-D profile (y, z) extruded along X from x0 to x1."""
    m = poly(pts).extrude(x1 - x0)
    return m.transform([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]])


def prism_xz(pts, y0, y1):
    """2-D profile (x, z) extruded along Y from y0 to y1."""
    m = poly(pts).extrude(y1 - y0)
    return m.transform([[1, 0, 0, 0], [0, 0, -1, y1], [0, 1, 0, 0]])


def union(parts):
    return M.batch_boolean(list(parts), manifold3d.OpType.Add)


def csk_hole(x, y, z_top, depth, shank=2.3, head=4.6, down=False):
    """Countersunk #8 / M4 wood-screw hole through a plate. down=True: head on the bottom face."""
    shaft = cyl(x, y, z_top - depth - 1, depth + 2, shank)
    cone_h = head - shank
    if down:
        cone = cyl(x, y, z_top - depth - 0.01, cone_h + 0.01, head + 0.01, shank)
    else:
        cone = cyl(x, y, z_top - cone_h, cone_h + 0.01, shank, head + 0.01)
    return shaft + cone


def u_slot(x, y0, y1, z_bottom, z_top, w):
    """Vertical U-slot (open at the top, round bottom) cut through a wall running along X."""
    r = w / 2
    return box(x - r, y0 - 1, z_bottom + r, x + r, y1 + 1, z_top + 1) + \
        prism_xz([(x + r * np.cos(a), z_bottom + r + r * np.sin(a)) for a in np.linspace(np.pi, 2 * np.pi, 33)] +
                 [(x + r, z_bottom + r + 0.01), (x - r, z_bottom + r + 0.01)], y0 - 1, y1 + 1)


# ---------------------------------------------------------------- 7-segment glyphs
SEG = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc",
       "7": "abc", "8": "abcdefg", "9": "abcdfg", "P": "abefg", "F": "aefg", "U": "bcdef", "E": "adefg",
       "S": "afgcd", "L": "def", "A": "abcefg", "C": "adef", "D": "bcdeg", "T": "defg", "N": "ceg"}


def glyph_cs(ch, h=8.0, s=1.3):
    """Character as a 2-D CrossSection, origin = bottom-left, width h/2 + s."""
    w = h / 2 + s
    segs = {"a": (0, h - s, w, h), "g": (0, h / 2 - s / 2, w, h / 2 + s / 2), "d": (0, 0, w, s),
            "f": (0, h / 2, s, h), "e": (0, 0, s, h / 2), "b": (w - s, h / 2, w, h), "c": (w - s, 0, w, h / 2)}
    parts = [CS.square([x1 - x0, y1 - y0]).translate([x0, y0]) for x0, y0, x1, y1 in (segs[k] for k in SEG[ch])]
    return CS.batch_boolean(parts, manifold3d.OpType.Add)


def text_cs(txt, h=8.0, s=1.3, gap=1.6):
    w = h / 2 + s
    out = [glyph_cs(c, h, s).translate([i * (w + gap), 0]) for i, c in enumerate(txt) if c != " "]
    total = len(txt) * (w + gap) - gap
    return CS.batch_boolean(out, manifold3d.OpType.Add), total


def text_on_front(txt, cx, y_face, z0, h=8.0, depth=0.8):
    """Raised text on a face that looks toward -Y (prints fine as a vertical face)."""
    cs, total = text_cs(txt, h)
    m = cs.translate([-total / 2, 0]).extrude(depth)
    # (u, v, w) -> (x = cx + u, y = y_face - w, z = z0 + v)
    return m.transform([[1, 0, 0, cx], [0, 0, -1, y_face + 0.01], [0, 1, 0, z0]])


def text_on_top(txt, cx, cy, z_top, h=8.0, depth=0.6):
    cs, total = text_cs(txt, h)
    return cs.translate([cx - total / 2, cy - h / 2]).extrude(depth).translate([0, 0, z_top - 0.01])


# ---------------------------------------------------------------- shared comb (used by dock + standalone)
COMB_T = 7.0      # wall thickness along Y
COMB_TOP = 34.0   # comb height above the comb's own floor
LOCK_PIN_R = 1.8
LOCK_HOLE_R = LOCK_PIN_R + 0.25


def comb_teeth_centers(xs):
    xs = sorted(xs)
    return [(a + b) / 2 for a, b in zip(xs, xs[1:])]


def comb(x_off, y0, z_floor, x_min, x_max, labels=True):
    """Cable comb wall from x_min..x_max (global), front face at y0, open-top U slots for every
    front connector. Returns (solid, list of lock-pin hole centres)."""
    y1 = y0 + COMB_T
    top = z_floor + COMB_TOP
    wall = rslab(x_min, y0, x_max, y1, z_floor, top, 2.0)
    xs = [x_off + x for _, x in FRONT_SLOTS]
    cuts = [u_slot(x, y0, y1, z_floor + 8.0, top, SLOT_W) for x in xs]
    holes = []
    for tx in comb_teeth_centers(xs) + [xs[0] - 13.0, xs[-1] + 13.0]:
        holes.append(tx)
        cuts.append(cyl(tx, y0 + COMB_T / 2, top - 7.0, 7.1, LOCK_HOLE_R))
    solid = wall - union(cuts)
    if labels:
        solid = solid + union(text_on_front(n, x_off + x, y0, z_floor + 1.6, h=5.0, depth=0.7)
                              for n, x in FRONT_SLOTS)
    return solid, [(h, y0 + COMB_T / 2, top) for h in holes]


def lock_bar(holes, x_min, x_max, name_y=0.0):
    """Bar that sits on top of the comb and closes all slots; pins print pointing up and the bar
    is flipped onto the comb. Generated at the origin."""
    w = COMB_T + 4.0
    bar = rslab(x_min, -w / 2, x_max, w / 2, 0, 4.0, 2.0)
    # pins with a 0.8 mm slit at the tip so they compress slightly into the holes
    pins = [cyl(x, 0, 3.99, 6.0, LOCK_PIN_R) - box(x - 3, -0.4, 7.5, x + 3, 0.4, 10.1) for x, _, _ in holes]
    return bar + union(pins)


# ---------------------------------------------------------------- A: RB5009 dock
def tie_arch(x, y, z0, span=18.0, width=8.0, h=4.0, tunnel="y"):
    """Strap / zip-tie anchor: a bridge; a 12-20 mm velcro strap (or a cable) passes through the
    tunnel in direction `tunnel`. span = clear tunnel width, h = clear height."""
    t = 2.0
    if tunnel == "y":   # legs separated along X, passage along Y
        legs = box(x - span / 2 - t, y - width / 2, z0, x - span / 2, y + width / 2, z0 + h + t) + \
            box(x + span / 2, y - width / 2, z0, x + span / 2 + t, y + width / 2, z0 + h + t)
        top = box(x - span / 2 - t, y - width / 2, z0 + h, x + span / 2 + t, y + width / 2, z0 + h + t)
    else:
        legs = box(x - width / 2, y - span / 2 - t, z0, x + width / 2, y - span / 2, z0 + h + t) + \
            box(x - width / 2, y + span / 2, z0, x + width / 2, y + span / 2 + t, z0 + h + t)
        top = box(x - width / 2, y - span / 2 - t, z0 + h, x + width / 2, y + span / 2 + t, z0 + h + t)
    return legs + top


def dock():
    W, D, T = 254.0, 300.0, 3.0
    rw, rd, _ = ROUTER
    px0 = (W - rw) / 2 - ROUTER_CLEAR                    # pocket incl. clearance
    px1 = (W + rw) / 2 + ROUTER_CLEAR
    py0, py1 = 82.0, 82.0 + rd + 2 * ROUTER_CLEAR
    rx0 = (W - rw) / 2                                   # router left end (nominal)
    rail_top = T + 3.5                                   # 3.5 mm air gap under the router

    plate = rslab(0, 0, W, D, 0, T, 12.0)
    # vent windows under the router (between rails) + material saving in the brick bay
    rails_x = np.linspace(px0 + 14, px1 - 14, 6)
    vents = [rslab(a + 5, py0 + 12, b - 5, py1 - 12, -1, T + 1, 3.0) for a, b in zip(rails_x, rails_x[1:])]
    plate = plate - union(vents)
    parts = [plate]
    parts += [box(x - 2.5, py0 + 4, T - 0.01, x + 2.5, py1 - 4, rail_top) for x in rails_x]

    # corner locators (L-shaped, 16 mm legs: sides stay open for the power terminal, tabs, air)
    lh = rail_top + 13.0
    for cx, sx in ((px0, -1), (px1, 1)):
        for cy, sy in ((py0, -1), (py1, 1)):
            xa, xb = sorted((cx, cx + sx * 4.0))
            ya, yb = sorted((cy, cy + sy * 4.0))
            leg_x = box(min(cx, cx - sx * 16), ya, T - 0.01, max(cx, cx - sx * 16), yb, lh)
            leg_y = box(xa, min(cy, cy - sy * 16), T - 0.01, xb, max(cy, cy - sy * 16), lh)
            post = box(xa, ya, T - 0.01, xb, yb, lh)
            # the front locators must not block the front panel: only the end-legs at the front
            parts += [post, leg_y] if sy < 0 else [post, leg_x, leg_y]
    # front stop is the comb zone; the router is held by the 4 corner posts

    # comb 45 mm in front of the router face (RJ45 plug + boot ~ 35-40 mm)
    comb_y = py0 - 52.0
    c, holes = comb(rx0, comb_y, T - 0.01, 12.0, W - 12.0)
    parts.append(c)

    # power lane on the left: DC cable runs from the brick bay along the left edge to the front
    lane_x0, lane_x1 = 3.0, px0 - 4.5
    parts.append(box(0.8, 18, T - 0.01, 3.0, D - 14, T + 12.0))                 # outer wall
    for y in np.arange(110, D - 30, 60):
        parts.append(tie_arch((lane_x0 + lane_x1) / 2, y, T - 0.01, span=lane_x1 - lane_x0 - 0.5,
                              width=6.0, h=6.0, tunnel="y"))

    # rear power-brick bay (open top: bricks and PLA both dislike heat)
    by0, by1 = py1 + 8.0, D - 3.0
    bay = rslab(px0, by0, W - 3.0, by1, T - 0.01, T + 34.0, 4.0) - \
        rslab(px0 + 2.4, by0 + 2.4, W - 5.4, by1 - 2.4, T, T + 40.0, 2.0)
    slots = [box(x, by0 - 1, T + 8, x + 6, by1 + 1, T + 26) for x in np.arange(px0 + 16, W - 20, 14)]
    slots += [box(W - 10, by0 + 12, T + 8, W + 1, by1 - 12, T + 26)]
    bay = bay - union(slots)
    bay = bay - box(px0 - 1, by0 + 10, T + 6, px0 + 3.4, by0 + 30, T + 40)     # cord exit to the power lane
    bay = bay - box(W - 50, by1 - 3, T + 14, W - 22, by1 + 1, T + 40)          # mains cord exit at the rear
    parts.append(bay)
    for x in (px0 + 45, (px0 + W) / 2, W - 45):
        parts.append(tie_arch(x, (by0 + by1) / 2, T - 0.01, span=20.0, width=8.0, h=4.0, tunnel="y"))

    # front zone: strap anchors to bundle the data cables after the comb (strap runs along X)
    for x in np.linspace(60, W - 60, 4):
        parts.append(tie_arch(x, comb_y - 15, T - 0.01, span=20.0, width=8.0, h=4.5, tunnel="x"))

    parts.append(text_on_front("DC", px0 + 30, by0, T + 24, h=6.0, depth=0.7))
    body = union(parts)
    lb = lock_bar(holes, 12.0, W - 12.0)
    return body, lb, holes


# ---------------------------------------------------------------- B: standalone comb
def standalone_comb():
    L = 240.0
    x_off = (L - ROUTER[0]) / 2
    base = rslab(0, -38, L, COMB_T + 6, 0, 4.0, 6.0)
    for x in (25.0, L / 2, L - 25.0):
        base = base - csk_hole(x, -22, 4.0, 4.0)
    # 4 recesses for the router's own 7 mm adhesive pads / any foam tape (0.6 mm deep)
    for x in (12.0, L - 12.0):
        base = base - cyl(x, -30, -0.01, 0.61, 4.2)
    c, holes = comb(x_off, 0.0, 3.99, 0.0, L)
    body = base + c
    for x in np.linspace(45, L - 45, 3):
        body = body + tie_arch(x, -10, 3.99, span=18.0, width=8.0, h=4.5, tunnel="x")
    return body, lock_bar(holes, 0.0, L), holes


# ---------------------------------------------------------------- C: twin-lane raceway
RW = 56.0          # outer width
RH = 24.0          # base height
RT = 2.4           # wall / floor
DATA_W = 30.0      # data lane inner width (~10-12 Cat6)
BARB = 0.6         # snap ridge on the base walls (lid skirt flexes ~0.6 mm: ~2 % strain in PLA)
LID_SKIRT = 8.5


def raceway_base_profile():
    # outer walls with an outward snap ridge at the top, U profile, divider
    y0, y1 = 0.0, RW
    pts = [(y0, 0), (y1, 0), (y1, RH - 6), (y1 + BARB, RH - 6 + BARB * 1.2), (y1 + BARB, RH - 3.2),
           (y1, RH - 2.4), (y1, RH), (y1 - RT, RH), (y1 - RT, RT), (y0 + RT, RT), (y0 + RT, RH),
           (y0, RH), (y0, RH - 2.4), (y0 - BARB, RH - 3.2), (y0 - BARB, RH - 6 + BARB * 1.2), (y0, RH - 6)]
    return pts


def raceway_base(L=300.0, exits=False):
    body = prism_yz(raceway_base_profile(), 0, L)
    div_y = RT + DATA_W
    body = body + box(0, div_y, RT - 0.01, L, div_y + 1.6, RH - 2.0)
    # screw holes in both lanes
    for x in np.arange(40, L, 110):
        body = body - csk_hole(x, RT + DATA_W / 2, RT, RT)
        body = body - csk_hole(x + 55 if x + 55 < L - 20 else x - 20, div_y + 1.6 + (RW - RT - div_y - 1.6) / 2, RT, RT)
    if exits:   # side exits for cables that leave mid-run (data side)
        for x in (L * 0.33, L * 0.66):
            body = body - box(x - 7, -2, RT + 4, x + 7, RT + 0.5, RH + 1)
    # labels inside the lanes
    body = body + text_on_top("LAN", 22, RT + DATA_W / 2, RT, h=6.0, depth=0.5)
    body = body + text_on_top("P", 22, div_y + 1.6 + 7.5, RT, h=6.0, depth=0.5)
    return body


def raceway_lid(L=300.0):
    """Printed upside down: top plate on the bed, skirts upward with inward snap barbs (45 deg)."""
    iw = RW + 2 * FIT          # inner width between skirts
    t = 2.0                    # top plate
    st = 1.6                   # skirt thickness
    skirt = LID_SKIRT
    k = FIT                    # barb reaches the wall surface: no interference once seated
    y0, y1 = -st, iw + st
    # seated, the barb sits just below the base's snap ridge (base z 15.5-17.9 vs ridge 18-21.6)
    pts = [(y0, 0), (y1, 0), (y1, t + skirt), (iw - k, t + skirt), (iw - k, t + skirt - 1.6),
           (iw, t + skirt - 1.6 - k * 1.3), (iw, t), (0, t), (0, t + skirt - 1.6 - k * 1.3),
           (k, t + skirt - 1.6), (k, t + skirt), (y0, t + skirt)]
    lid = prism_yz(pts, 0, L)
    # finger notch at one end to pry the lid off
    lid = lid - box(-1, iw / 2 - 10, -1, 3.0, iw / 2 + 10, 1.0)
    return lid


def raceway_joiner():
    """U clip that sits inside the LAN lane across a seam between two bases (friction fit)."""
    w = DATA_W - 2 * FIT
    return rslab(0, 0, 40, w, 0, 1.6, 2.0) + box(0, 0, 0, 40, 1.4, 10) + box(0, w - 1.4, 0, 40, w, 10)


# ---------------------------------------------------------------- D: under-desk basket
def basket():
    L, Wd, H, t = 330.0, 130.0, 80.0, 2.6
    outer = rslab(0, 0, L, Wd, 0, H, 8.0)
    inner = rslab(t, t, L - t, Wd - t, t, H + 1, 8.0 - t)
    b = outer - inner
    cuts = []
    # floor vents (also drain dust) – leave a solid margin
    for x in np.arange(22, L - 22, 16):
        cuts.append(rslab(x, 22, x + 8, Wd - 22, -1, t + 1, 2.0))
    # wall vents: tall slots with 45-degree pointed tops (no bridging)
    for x in np.arange(30, L - 30, 16):
        for (ya, yb) in ((-1, t + 1), (Wd - t - 1, Wd + 1)):
            slot = prism_xz([(x, 14), (x + 7, 14), (x + 7, H - 26), (x + 3.5, H - 22.5), (x, H - 26)], ya, yb)
            cuts.append(slot)
    for y in np.arange(30, Wd - 30, 16):
        for (xa, xb) in ((-1, t + 1), (L - t - 1, L + 1)):
            cuts.append(prism_yz([(y, 14), (y + 7, 14), (y + 7, H - 26), (y + 3.5, H - 22.5), (y, H - 26)], xa, xb))
    # cord entry notches in the rear wall (y = Wd side) and both ends
    for x in (60.0, L / 2, L - 60.0):
        cuts.append(box(x - 14, Wd - t - 1, H - 22, x + 14, Wd + 1, H + 1))
    for x in (-1, L - t - 1):
        cuts.append(box(x, Wd / 2 - 16, H - 26, x + t + 2, Wd / 2 + 16, H + 1))
    b = b - union(cuts)
    # mounting flanges on both long sides with a 45-degree solid wedge underneath (no supports)
    F = 20.0
    for side in (0, 1):
        if side == 0:
            pts = [(0.01, H - F - 3), (0.01, H), (-F, H), (-F, H - 3)]
        else:
            pts = [(Wd - 0.01, H - F - 3), (Wd - 0.01, H), (Wd + F, H), (Wd + F, H - 3)]
        for x in (45.0, L / 2, L - 45.0):
            fl = prism_yz(pts, x - 22, x + 22)
            yy = -F / 2 if side == 0 else Wd + F / 2
            # screw goes up into the desk: head under the flange; deep counterbore from below
            fl = fl - cyl(x, yy, H - 30, 31, 2.4) - cyl(x, yy, H - 30, 30 - 3.2, 5.0)
            b = b + fl
    # strap anchors on the floor for the power strip
    for x in (L * 0.25, L * 0.5, L * 0.75):
        b = b + tie_arch(x, Wd / 2, t - 0.01, span=24.0, width=8.0, h=4.0, tunnel="y")
    return b


# ---------------------------------------------------------------- E: tags + adhesive holders
def cable_tag(label):
    r_in = CABLE_TAG_D / 2
    t = 1.6
    open_w = 5.0
    ring = CS.circle(r_in + t) - CS.circle(r_in)
    gap = CS.square([open_w, r_in + t + 1]).translate([-open_w / 2, 0])          # opening on +Y
    ring = ring - gap
    flag = CS.square([18.0, 2.0]).translate([r_in + t - 0.6, -1.0])
    shape = ring + flag
    h = 9.0
    tag = shape.extrude(h)
    # label raised on the flag's -Y face
    cs, total = text_cs(label, h=6.0, s=1.1, gap=1.2)
    txt = cs.translate([-total / 2, 0]).extrude(0.7)
    cx = r_in + t - 0.6 + 9.0 + 1.0
    txt = txt.transform([[1, 0, 0, cx], [0, 0, -1, -1.0 + 0.01], [0, 1, 0, 1.5]])
    return tag + txt


def tag_sheet():
    labels = ["1", "2", "3", "4", "5", "6", "7", "8", "F", "U", "P", "P"]
    tags = []
    for i, lab in enumerate(labels):
        tags.append(cable_tag(lab).translate([(i % 4) * 34.0, (i // 4) * 18.0, 0]))
    return union(tags)


def adhesive_holder(n=4, d=6.4):
    """Stick-on block: n snap channels for cables d mm, 3M/VHB tape on the flat back."""
    pitch = d + 5.0
    L = n * pitch + 4
    hgt = 5.2 + d + 2.0
    blk = rslab(0, 0, L, 16.0, 0, hgt, 2.0)
    for i in range(n):
        x = 2 + pitch / 2 + i * pitch
        ch = M.cylinder(18.0, d / 2 + 0.2).transform([[1, 0, 0, x], [0, 0, -1, 17.0], [0, 1, 0, 5.2 + d / 2]])
        mouth = box(x - (d * 0.78) / 2, -1, 5.2 + d / 2, x + (d * 0.78) / 2, 17, hgt + 1)
        blk = blk - ch - mouth
    return blk


def holder_sheet():
    hs = [adhesive_holder(4).translate([0, i * 22.0, 0]) for i in range(3)]
    hs += [adhesive_holder(2, 8.4).translate([60.0, i * 22.0, 0]) for i in range(3)]
    return union(hs)


# ---------------------------------------------------------------- F: fit-test coupons
def coupons():
    """30-minute plate to test the fits before a long print: 3-slot comb section + lock bar,
    40 mm raceway base + lid (snap), one cable tag."""
    eth1 = dict(FRONT_SLOTS)["1"]
    x_off = 12.0 - eth1
    foot = rslab(0, -14, 60, COMB_T, 0, 3.0, 3.0)
    c, holes = comb(x_off, 0.0, 2.99, 0.0, 60.0, labels=False)
    holes = [h for h in holes if 3.0 < h[0] < 57.0]
    test_comb = foot + c + union(text_on_front(n, x_off + x, 0.0, 4.0, h=5.0, depth=0.7)
                                 for n, x in FRONT_SLOTS if 0 < x_off + x < 60)
    parts = [test_comb, lock_bar(holes, 0.0, 60.0).translate([0, 24, 0]),
             raceway_base(L=40.0).translate([70, -14, 0]), raceway_lid(L=40.0).translate([120, -14, 0]),
             cable_tag("1").translate([10, 40, 0])]
    return union(parts)


# ---------------------------------------------------------------- export
def export(name, man, desc):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    tm = trimesh.Trimesh(v, f, process=False)   # manifold3d output is already manifold
    zmin = tm.bounds[0][2]
    tm.apply_translation([0, 0, -zmin])
    path = os.path.join(OUT, name + ".stl")
    tm.export(path)
    ext = (tm.bounds[1] - tm.bounds[0]).round(1).tolist()
    info = {"name": name, "file": name + ".stl", "desc": desc, "extent_mm": ext,
            "volume_cm3": round(tm.volume / 1000, 1), "watertight": bool(tm.is_watertight),
            "triangles": int(len(tm.faces)), "genus_ok": man.status().name if hasattr(man.status(), 'name') else str(man.status())}
    print(json.dumps(info))
    return info


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = []
    d, dl, _ = dock()
    parts.append(export("A1_rb5009_dock", d, "RB5009 dock: router cradle, port comb, power lane, brick bay"))
    parts.append(export("A2_dock_lock_bar", dl, "Lock bar for the dock comb (flip onto the comb)"))
    c, cl, _ = standalone_comb()
    parts.append(export("B1_port_comb", c, "Standalone RB5009 port comb with screw/tape foot"))
    parts.append(export("B2_port_comb_lock_bar", cl, "Lock bar for the standalone comb"))
    parts.append(export("C1_raceway_base", raceway_base(), "Twin-lane raceway base, 300 mm (LAN + power)"))
    parts.append(export("C2_raceway_base_exits", raceway_base(exits=True), "Raceway base with 2 side exits"))
    parts.append(export("C3_raceway_lid", raceway_lid(), "Snap-on raceway lid, 300 mm (print upside down)"))
    parts.append(export("C4_raceway_joiner", raceway_joiner(), "Seam joiner clip (sits inside the LAN lane)"))
    parts.append(export("D1_underdesk_basket", basket(), "Under-desk basket for power strip, brick and slack"))
    parts.append(export("E1_cable_tags", tag_sheet(), "12 snap-on cable tags: 1-8, F (SFP), U (USB), P (power)"))
    parts.append(export("E2_adhesive_holders", holder_sheet(), "6 stick-on cable holders (4x6 mm, 2x8 mm)"))
    parts.append(export("F1_fit_test", coupons(), "Fit-test plate: comb section + lock bar, raceway snap, tag"))
    with open(os.path.join(OUT, "parts.json"), "w") as f:
        json.dump({"router_slots": FRONT_SLOTS, "parts": parts}, f, indent=1)


if __name__ == "__main__":
    main()
