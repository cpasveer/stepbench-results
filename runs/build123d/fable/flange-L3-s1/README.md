# Design the connecting part

Task flange-L3-s1. Units: millimetres.

Two parts are fixed in space: a round flange and a rectangular block. Their mounting faces stand upright, roughly facing each other but turned at an angle, the block offset to one side and in height. Nothing stands between them; the arrangement stands at an arbitrary position, turned arbitrarily about the vertical.

Design **one solid part** that bolts to both fixed parts. It will be printed in PLA on a common filament printer (0.4 mm nozzle, 0.2 mm layers), without support material, and used as it comes off the printer.

**Make the best part you can.** Beyond the requirements below, how the part
looks and how it is shaped are your judgement as the designer. Aim for the
part an experienced mechanical designer would be proud of.

Save it as `output.step` in this folder. `submission.py` is a build123d
starting point; you may replace it entirely. Run it with the Python your
environment provides (it has build123d), then check the part with the
`jointbench` command:

    python submission.py                                     # writes output.step
    jointbench verify . --by <your name> --tool <the CAD tool you used>

From a checkout of the kit, put `uv run --project <the kit folder>` before
both commands.

## The fixed parts

**A** -- flat round mount 74.5 mm dia: 6x M5 equally spaced on a 60 mm bolt circle.
Mounting face: a flat round face 74.5 mm dia through (95.00, -91.00, 5.50), outward normal (0.74, 0.67, 0.00). The part's body lies behind the face; the space in front of it is yours.
Fixed geometry: `fixed_A.step` (world coordinates).

**B** -- flat rectangular mount 46 x 96 mm: 4x M6 in 2 x 2 grid at 30 x 80 mm.
Mounting face: a 46 x 96 mm flat face through (132.07, -30.75, -18.00), outward normal (-0.48, -0.88, 0.00). The part's body lies behind the face; the space in front of it is yours.
Fixed geometry: `fixed_B.step` (world coordinates).

## The bolts

Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped
hole in the fixed part and is put in from your side, so its head bears on
your part. The axis points from the fixed part into your part.

| bolt | size | position on the surface | axis | clearance hole dia | grip |
|---|---|---|---|---|---|
| A1 | M5 | (105.11, -102.08, 31.48) | (0.74, 0.67, 0.00) | 5.3-5.8 | 3.75-25 |
| A2 | M5 | (115.23, -113.15, 5.50) | (0.74, 0.67, 0.00) | 5.3-5.8 | 3.75-25 |
| A3 | M5 | (105.11, -102.08, -20.48) | (0.74, 0.67, 0.00) | 5.3-5.8 | 3.75-25 |
| A4 | M5 | (84.89, -79.92, -20.48) | (0.74, 0.67, 0.00) | 5.3-5.8 | 3.75-25 |
| A5 | M5 | (74.77, -68.85, 5.50) | (0.74, 0.67, 0.00) | 5.3-5.8 | 3.75-25 |
| A6 | M5 | (84.89, -79.92, 31.48) | (0.74, 0.67, 0.00) | 5.3-5.8 | 3.75-25 |
| B1 | M6 | (118.90, -23.57, -58.00) | (-0.48, -0.88, 0.00) | 6.4-7 | 4.5-25 |
| B2 | M6 | (145.24, -37.93, -58.00) | (-0.48, -0.88, 0.00) | 6.4-7 | 4.5-25 |
| B3 | M6 | (118.90, -23.57, 22.00) | (-0.48, -0.88, 0.00) | 6.4-7 | 4.5-25 |
| B4 | M6 | (145.24, -37.93, 22.00) | (-0.48, -0.88, 0.00) | 6.4-7 | 4.5-25 |

*Grip* is the thickness of your part under the head: the distance along
the axis from the fixed part's surface to where the hole ends.

## What is checked

Your part passes when it meets every requirement below.

- `FILE_VALID` -- `output.step` holds exactly one valid, closed solid.
- `HOLES` -- a through hole on every bolt axis, within 0.1 mm, diameter in the clearance range, grip in range.
- `SEATING` -- your part lies on each fixed part around every bolt, covering at least 75% of the ring from the hole out to the head diameter (stand-off at most 0.05 mm).
- `NO_INTERFERENCE` -- your part does not overlap any fixed part (limit 0.01 mm3), and each bolt has room: its shank, and its head with 0.5 mm clearance around and 1 mm above.
- `KEY_TURN` -- every bolt can be turned with its standard L-shaped hex key (ISO 2936), not just reached: the short leg in the socket, the long leg sticking out sideways, the key leaning at most 25 degrees off the bolt axis (a ball-end key), it must swing 60 degrees round the bolt without touching your part, the fixed parts or any obstacle. Keys (M5: 4 mm key, legs 25 and 70 mm; M6: 5 mm key, legs 28 and 80 mm; lengths over the outside of the bend).
- `SEAT_INSIDE` -- where your part lies on a mounting face it stays at least 1 mm inside that face's outline (at most 0.5 mm2 of seat beyond that line): no larger than the part it is bolted to, and not flush to its edge. Material that does not touch the mounting face is free.
- `CLEAN` -- the part is cleanly modelled, as an engineer would accept the file: no sliver faces (min 0.25 mm); no tiny faces (min 0.1 mm2); no very short edges (min 0.1 mm); no hairline steps between faces meant to be flush (min 0.2 mm); no knife edges or thin wedges (min 5 deg); no near-flat creases (min 2 deg); faces on the same surface are merged (max 0 pairs); no near-coincident corners (min 0.05 mm); no stubs: members end at the faces they join (min 5 mm); no shallow joints (min 45 deg). The report shows these as `Q_...` lines.
- `TOOL_ACCESS` -- above every bolt head, along its axis, a cylinder of the head diameter and 30 mm long is free of your part and the fixed parts: a straight hex key can reach the bolt.
- `MANUFACTURING` -- the part can be made as stated above, as it is, with no further work. The report shows this as `FDM_...` lines.
- `STIFFNESS` -- Fixed part B carries 0.8 kg (7.85 N). With A held, that weight may move B by at most 0.298 mm, whichever way it pulls (a turning load counts as the weight at the 74.54 mm span): at least 26.3 N/mm, in PLA, filament printed. How it is analysed: A is held rigid where your part touches it (its seat and bolt holes); B's load enters your part through B's seat and bolt holes, as through a rigid washer. The span is measured from the centre of A's mounting face to the centre of B's mounting face. The checker resolves +-2%; a result well clear of the limit may be settled at a first, coarser analysis (+-15%), and the report says which. A result inside the band counts as a pass and is flagged.
- **Score** -- among parts that meet every requirement above, the one that uses the least material wins: the plastic the printer lays down: walls of 3 x 0.4 mm on the sides, 4 solid layers of 0.2 mm on top and bottom faces, and 20% infill inside, each part as printed in the direction the checker finds for it. Stiffness beyond the requirement earns nothing.

The checker may print further lines (section, volume). They are information, not requirements.

This folder may hold `_solution/` with the generator's reference answer, for
practice. A folder handed out for a recorded run never contains it.
