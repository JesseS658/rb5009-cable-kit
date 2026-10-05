# RB5009 Cable Kit

Cable management for a MikroTik RB5009UG+S+ printed on a QIDI MAX 4 in ELEGOO PLA.
Click any file in `stl/` to see it in 3D; use the download button (↓ "Download raw file") to save it.
Every part prints in the orientation it is saved in, without supports.

Times and weights are OrcaSlicer 2.4.2 estimates (0.20 mm Standard @ X-Max 4, Textured PEI, PLA 220 °C / 55 °C).

| Code | Part | File | Size, mm | Print time | Filament |
|---|---|---|---|---|---|
| A | Router dock | `stl/A1_rb5009_dock.stl` | 254 × 300 × 37 | 6 h 51 min | 228 g |
| A | Dock lock bar (flip onto the comb) | `stl/A2_dock_lock_bar.stl` | 230 × 11 × 10 | 25 min | 8 g |
| B | Port comb | `stl/B1_port_comb.stl` | 240 × 51 × 38 | 2 h 07 min | 56 g |
| B | Comb lock bar | `stl/B2_port_comb_lock_bar.stl` | 240 × 11 × 10 | 26 min | 8 g |
| C | Raceway base, 300 mm | `stl/C1_raceway_base.stl` | 300 × 57 × 24 | 1 h 56 min | 80 g |
| C | Raceway base with side exits | `stl/C2_raceway_base_exits.stl` | 300 × 57 × 24 | 1 h 57 min | 78 g |
| C | Raceway lid (prints upside down) | `stl/C3_raceway_lid.stl` | 300 × 60 × 10.5 | 1 h 07 min | 47 g |
| C | Raceway joiner | `stl/C4_raceway_joiner.stl` | 40 × 29 × 10 | 17 min | 3 g |
| D | Under-desk basket | `stl/D1_underdesk_basket.stl` | 330 × 170 × 80 | 8 h 27 min | 237 g |
| E | 12 cable tags (1–8, F, U, P, P) | `stl/E1_cable_tags.stl` | 129 × 45 × 9 | 30 min | 8 g |
| E | 6 stick-on holders | `stl/E2_adhesive_holders.stl` | 91 × 60 × 16 | 1 h 05 min | 23 g |
| F | Fit-test plate (print first) | `stl/F1_fit_test.stl` | 160 × 60 × 37 | 1 h 14 min | 29 g |

Comb slot labels: P = DC power, F = SFP+, U = USB, 1–8 = Ethernet ports.
Port positions are estimated (standard 15.88 mm jack pitch); print F first and hold it against ports 1–3.

`source/make_designs.py` regenerates every part (Python, manifold3d + trimesh).
