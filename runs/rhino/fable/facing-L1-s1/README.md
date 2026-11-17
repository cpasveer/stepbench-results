# Design the connecting part

Task facing-L1-s1. Units: millimetres.

Two parts are fixed in space, one above the other. Their mounting faces are horizontal and face each other, with nothing in between; the arrangement stands at an arbitrary position, turned arbitrarily about the vertical.

Design **one solid part** that bolts to both fixed parts.

**Make the best part you can.** Beyond the requirements below, how the part
looks and how it is shaped are your judgement as the designer. Aim for the
part an experienced mechanical designer would be proud of.

Save it as `output.step` in this folder. `submission.py` is a Rhino
starting point (a Python 3 script using RhinoCommon); you may replace it
entirely. Run it inside Rhino with `run_rhino.sh`, which starts the script with
Rhino's command-line tool `rhinocode`, waits until it has finished and prints
its log (rhinocode itself returns before the script ends and does not show its
output). Then check the part with the `jointbench` command:

    sh run_rhino.sh                                          # writes output.step
    jointbench verify . --by <your name> --tool rhino

## The fixed parts

**A** -- flat square mount 73 mm across: 4x M4 on a 60 mm square.
Mounting face: a 73 x 73 mm flat face through (-126.00, 127.50, -94.00), outward normal (0.00, 0.00, 1.00). The part's body lies behind the face; the space in front of it is yours.
Fixed geometry: `fixed_A.step` (world coordinates).

**B** -- flat round mount 84.5 mm dia: 3x M5 equally spaced on a 70 mm bolt circle.
Mounting face: a flat round face 84.5 mm dia through (-124.18, 135.11, -25.50), outward normal (0.00, 0.00, -1.00). The part's body lies behind the face; the space in front of it is yours.
Fixed geometry: `fixed_B.step` (world coordinates).

## The bolts

Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped
hole in the fixed part and is put in from your side, so its head bears on
your part. The axis points from the fixed part into your part.

| bolt | size | position on the surface | axis | clearance hole dia | grip |
|---|---|---|---|---|---|
| A1 | M4 | (-167.78, 134.87, -94.00) | (0.00, 0.00, 1.00) | 4.3-4.8 | 3-25 |
| A2 | M4 | (-133.37, 85.72, -94.00) | (0.00, 0.00, 1.00) | 4.3-4.8 | 3-25 |
| A3 | M4 | (-84.22, 120.13, -94.00) | (0.00, 0.00, 1.00) | 4.3-4.8 | 3-25 |
| A4 | M4 | (-118.63, 169.28, -94.00) | (0.00, 0.00, 1.00) | 4.3-4.8 | 3-25 |
| B1 | M5 | (-150.99, 112.61, -25.50) | (0.00, 0.00, -1.00) | 5.3-5.8 | 3.75-25 |
| B2 | M5 | (-130.26, 169.58, -25.50) | (0.00, 0.00, -1.00) | 5.3-5.8 | 3.75-25 |
| B3 | M5 | (-91.29, 123.14, -25.50) | (0.00, 0.00, -1.00) | 5.3-5.8 | 3.75-25 |

*Grip* is the thickness of your part under the head: the distance along
the axis from the fixed part's surface to where the hole ends.

## What is checked

Your part passes when it meets every requirement below.

- `FILE_VALID` -- `output.step` holds exactly one valid, closed solid.
- `HOLES` -- a through hole on every bolt axis, within 0.1 mm, diameter in the clearance range, grip in range.
- `SEATING` -- your part lies on each fixed part around every bolt, covering at least 75% of the ring from the hole out to the head diameter (stand-off at most 0.05 mm).
- `NO_INTERFERENCE` -- your part does not overlap any fixed part (limit 0.01 mm3), and each bolt has room: its shank, and its head with 0.5 mm clearance around and 1 mm above.
- `KEY_TURN` -- every bolt can be turned with its standard L-shaped hex key (ISO 2936), not just reached: the short leg in the socket, the long leg sticking out sideways, the key leaning at most 25 degrees off the bolt axis (a ball-end key), it must swing 60 degrees round the bolt without touching your part, the fixed parts or any obstacle. Keys (M4: 3 mm key, legs 20 and 63 mm; M5: 4 mm key, legs 25 and 70 mm; lengths over the outside of the bend).
- `SEAT_INSIDE` -- where your part lies on a mounting face it stays at least 1 mm inside that face's outline (at most 0.5 mm2 of seat beyond that line): no larger than the part it is bolted to, and not flush to its edge. Material that does not touch the mounting face is free.
- `CLEAN` -- the part is cleanly modelled, as an engineer would accept the file: no sliver faces (min 0.25 mm); no tiny faces (min 0.1 mm2); no very short edges (min 0.1 mm); no hairline steps between faces meant to be flush (min 0.2 mm); no knife edges or thin wedges (min 5 deg); no near-flat creases (min 2 deg); faces on the same surface are merged (max 0 pairs); no near-coincident corners (min 0.05 mm); no stubs: members end at the faces they join (min 5 mm); no shallow joints (min 45 deg). The report shows these as `Q_...` lines.

The checker may print further lines (section, volume). They are information, not requirements.

This folder may hold `_solution/` with the generator's reference answer, for
practice. A folder handed out for a recorded run never contains it.
