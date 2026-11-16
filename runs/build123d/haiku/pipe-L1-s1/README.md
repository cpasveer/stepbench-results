# Design the connecting part

Task pipe-L1-s1. Units: millimetres.

Two parts are fixed in space at an angle to each other. One is a round pipe with its bolts screwed straight into it, in two rows round it; the other has a flat mounting face. Nothing stands between them.

Design **one solid part** that bolts to both fixed parts.

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

**A** -- round pipe 68 mm outside diameter: 4x M5 screwed straight into it, square to its surface. The pipe's axis runs through (21.00, 136.50, -7.00) along (0.25, 0.59, 0.77); its outside radius is 34 mm and it is 260 mm long, centred on its bolts. Your part bears on the pipe's round surface: there is no flat face here.
Fixed geometry: `fixed_A.step` (world coordinates).

**B** -- flat square mount 38 mm across: 4x M4 on a 25 mm square.
Mounting face: a 38 x 38 mm flat face through (19.55, 49.69, 52.25), outward normal (0.39, 0.92, -0.00). The part's body lies behind the face; the space in front of it is yours.
Fixed geometry: `fixed_B.step` (world coordinates).

## The bolts

Every bolt is an ISO 4762 socket head cap screw. It threads into a tapped
hole in the fixed part and is put in from your side, so its head bears on
your part. The axis points from the fixed part into your part.

| bolt | size | position on the surface | axis | clearance hole dia | grip |
|---|---|---|---|---|---|
| A1 | M5 | (40.76, 104.04, -11.25) | (0.71, -0.65, 0.27) | 5.3-5.8 | 3.75-25 |
| A2 | M5 | (49.47, 124.69, 15.63) | (0.71, -0.65, 0.27) | 5.3-5.8 | 3.75-25 |
| A3 | M5 | (3.82, 103.35, 1.25) | (-0.38, -0.67, 0.64) | 5.3-5.8 | 3.75-25 |
| A4 | M5 | (12.53, 124.00, 28.13) | (-0.38, -0.67, 0.64) | 5.3-5.8 | 3.75-25 |
| B1 | M4 | (8.03, 54.54, 39.75) | (0.39, 0.92, -0.00) | 4.3-4.8 | 3-25 |
| B2 | M4 | (8.03, 54.54, 64.75) | (0.39, 0.92, -0.00) | 4.3-4.8 | 3-25 |
| B3 | M4 | (31.07, 44.83, 64.75) | (0.39, 0.92, -0.00) | 4.3-4.8 | 3-25 |
| B4 | M4 | (31.07, 44.83, 39.75) | (0.39, 0.92, -0.00) | 4.3-4.8 | 3-25 |

*Grip* is the thickness of your part under the head: the distance along
the axis from the fixed part's surface to where the hole ends.

## What is checked

Your part passes when it meets every requirement below.

- `FILE_VALID` -- `output.step` holds exactly one valid, closed solid.
- `HOLES` -- a through hole on every bolt axis, within 0.1 mm, diameter in the clearance range, grip in range.
- `SEATING` -- your part lies on each fixed part around every bolt, covering at least 75% of the ring from the hole out to the head diameter (stand-off at most 0.05 mm): on a flat face a flat face, on the pipe a face following its round surface.
- `NO_INTERFERENCE` -- your part does not overlap any fixed part (limit 0.01 mm3), and each bolt has room: its shank, and its head with 0.5 mm clearance around and 1 mm above.
- `KEY_TURN` -- every bolt can be turned with its standard L-shaped hex key (ISO 2936), not just reached: the short leg in the socket, the long leg sticking out sideways, the key leaning at most 25 degrees off the bolt axis (a ball-end key), it must swing 60 degrees round the bolt without touching your part, the fixed parts or any obstacle. Keys (M4: 3 mm key, legs 20 and 63 mm; M5: 4 mm key, legs 25 and 70 mm; lengths over the outside of the bend).
- `SEAT_INSIDE` -- where your part lies on a mounting face it stays at least 1 mm inside that face's outline (at most 0.5 mm2 of seat beyond that line): no larger than the part it is bolted to, and not flush to its edge. Material that does not touch the mounting face is free.
- `CLEAN` -- the part is cleanly modelled, as an engineer would accept the file: no sliver faces (min 0.25 mm); no tiny faces (min 0.1 mm2); no very short edges (min 0.1 mm); no hairline steps between faces meant to be flush (min 0.2 mm); no knife edges or thin wedges (min 5 deg); no near-flat creases (min 2 deg); faces on the same surface are merged (max 0 pairs); no near-coincident corners (min 0.05 mm); no stubs: members end at the faces they join (min 5 mm); no shallow joints (min 45 deg). The report shows these as `Q_...` lines.

The checker may print further lines (section, volume). They are information, not requirements.

This folder may hold `_solution/` with the generator's reference answer, for
practice. A folder handed out for a recorded run never contains it.
