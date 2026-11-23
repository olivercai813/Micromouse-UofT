
# Micromouse maze: cost estimate

Cost of building a practice maze for the UofT Micromouse team, using laser-cut
plywood walls, 3D-printed posts and a cardboard (or plywood) floor.

Prices are from the makerspace material list and online retailers in November 2026.
Prices from the material list are assumed to be **CAD**. Check before ordering.

---

## 1. Maze dimensions (competition standard)

| Item | Size |
|---|---|
| Maze | 16 × 16 cells |
| Cell | 180 mm × 180 mm |
| Walls | 50 mm high, 12 mm thick (competition) |
| Posts | 12 × 12 × 50 mm, one at every cell corner |
| Whole maze | about 2.9 m × 2.9 m (≈ 114" × 114") |

**Counts for the full 16 × 16 maze**

| Item | How it's counted | Count |
|---|---|---|
| Wall positions | 17 lines × 16 segments × 2 directions | **544** (64 outer + 480 inner) |
| Posts | 17 × 17 grid of corners | **289** |
| Walls in one typical maze layout | roughly 55–60% of positions | **~300** |

---

## 2. Walls: laser-cut plywood

**Material:** Plywood Birch 1/8" – Laser, 18" × 24" sheet, **$8.13 per sheet**, *Final Sale*.

**Wall piece size:** 176 × 50 mm (168 mm gap between posts + 4 mm into each post's slot).
An earlier estimate used 180 × 50 mm; both give the same count per sheet.

**Pieces per sheet** (sheet = 457 × 610 mm):
- 3 pieces along the 24" side × 9 rows along the 18" side = **27**
- (up to ~29 using the leftover strip; **27 assumed** to leave room for laser spacing and margins)

| Option | Wall pieces | Sheets | Cost |
|---|---|---|---|
| Full set (any layout possible) | 544 | 21 | **$170.73** |
| Full set + ~10% spares | ~600 | 23 | **$186.99** |
| One maze layout + spares | ~330 | 13 | **$105.69** |
| Outer walls only | 64 | 3 | **$24.39** |

> Note: 1/8" plywood is ~3.2 mm thick, much thinner than the 12 mm competition walls,
> so corridors are slightly wider than in competition. Fine for practice.

---

## 3. Floor: cardboard

**Floor area:** about 114" × 114" (2.9 m square).

**Sheets needed (48" × 96" corrugated):** **4**
- 3 sheets side by side cover 144" × 96"
- 1 sheet cut into 18"-wide strips covers the remaining 18" × 114" edge

| Source | Price | Notes |
|---|---|---|
| Staples, 48" × 96" kraft corrugated, 5-sheet bundle | $30.79 USD ≈ **$40–45 CAD** (conversion estimate) | 1 bundle covers the floor |
| Uline Canada, 48" × 96" corrugated pads | $6.70 each | Only sold in bales of 250, not practical |
| Appliance / bike boxes from local stores | **Free** | Sizes vary |

> Competition floors are flat, wooden and painted **matte black**. Tape cardboard seams flat
> and consider matte black paint so IR sensor tuning carries over.

---

## 4. Totals

| Option | Walls | Floor | **Total** |
|---|---|---|---|
| Plywood walls (full set + spares) + cardboard floor | $187 | ~$45 | **≈ $232** |
| Plywood walls (one layout + spares) + cardboard floor | $106 | ~$45 | **≈ $151** |
| Plywood walls only, full set + spares (use an existing floor) | $187 | $0 | **≈ $187** |
| Plywood walls only, one layout + spares | $106 | $0 | **≈ $106** |
| Plywood outer walls only | $24 | $0 | **≈ $24** |
| **All cardboard** (walls + floor): 6–7 sheets = 2 bundles | – | – | **≈ $62 USD ≈ $85 CAD** |
| All cardboard from free boxes | – | – | **$0** |

**All-cardboard wall count:** one 48" × 96" sheet cut into 50 mm strips gives 24 strips,
each fitting 13 wall pieces, so ~300 walls per sheet (~280 after waste). The full set of
544 walls needs 2 sheets (3 with spares), plus 4 floor sheets.

> Not included above: **posts** (see section 6), **laser-cutting time** if the makerspace
> charges for it, **paint** (white walls, red tops, matte black floor), and **glue/tape**.

---

## 5. Smaller test mazes (recommended to start)

Keep **full-size 180 mm cells** (the mouse and sensors are designed for them) and build
fewer cells. Wall counts below are *all* positions, so any layout can be built.

| Size | Good for | Posts | Walls | Plywood sheets | Wall cost | Floor size |
|---|---|---|---|---|---|---|
| 1 × 6 corridor | Driving straight, speed control, side sensors | 14 | 14 | 1 | **$8.13** | 0.2 × 1.1 m |
| 4 × 4 (one floor tile) | Turns, wall detection, short paths | 25 | 40 | 2 | **$16.26** | 0.73 m square |
| 6 × 6 | First maze solving, dead ends | 49 | 84 | 4 | **$32.52** | 1.1 m square |
| 8 × 8 (quarter maze) | Flood fill, best route, speed runs | 81 | 144 | 6 | **$48.78** | 1.45 m square |
| 16 × 16 (full) | Competition practice | 289 | 544 | 21 | **$170.73** | 2.9 m square |

**Suggested path:** build a **4 × 4** first, grow to **8 × 8** by adding three more floor
tiles, and build the full 16 × 16 only before competitions. Nothing built earlier is wasted.

---

## 6. Posts: 3D-printed (material estimate)

*This is a new estimate, not part of the earlier calculations.*

| Item | Value |
|---|---|
| Post design | 12 × 12 × 50 mm, 4 slots (3.4 mm wide, 4 mm deep), 6 mm × 5 mm peg |
| Volume per post | ≈ 4.6 cm³ |
| PLA per post (100% infill, 1.24 g/cm³) | ≈ **5.7 g** |

| Maze | Posts | PLA needed |
|---|---|---|
| 4 × 4 | 25 | ≈ 0.14 kg |
| 8 × 8 | 81 | ≈ 0.46 kg |
| 16 × 16 | 289 | ≈ 1.65 kg |

Cost depends on the makerspace's 3D-printing price (not listed in the material sheet).
As an example only, at about $25 CAD per kg of PLA, the full set of posts is roughly
**$40** of filament. Add ~10% for failed prints and test pieces.

---

## 7. Assumptions

- Plywood price and sheet size from the makerspace material list; assumed CAD; *Final Sale*.
- 27 wall pieces per sheet, allowing for laser spacing and margins.
- Cardboard USD→CAD conversion is approximate; Canadian store prices may differ.
- "Typical layout" ≈ 300 walls; a specific maze may need more or fewer.
- Floor tiles (if made in MDF/plywood instead of cardboard) are not costed here.

## Sources

- Makerspace material list: Plywood Birch 1/8" – Laser, 18" × 24", $8.13, Final Sale
- [Staples – 48" × 96" kraft corrugated sheets, 5/bundle](https://www.staples.com/si-products-corrugate-sheet-48-x-96-32-ect-kraft-5-bundle-bssp4896/product_681495)
- [Uline Canada – 48 × 96" 32 ECT corrugated pads](https://uline.ca/Product/Detail/S-20458/Corrugated-Pads/48-x-96-32-ECT-Corrugated-Pads)
- [UKMARS – Micromouse Classic rules](https://ukmars.org/contests/contest-rules/micromouse-classic) (maze dimensions)
