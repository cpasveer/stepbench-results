# STEP.bench v1 results

Every task, every run and every graded attempt of STEP.bench version 1, the
proof of concept. The results are shown, with every part in 3D, at
[stepbench.dev](https://stepbench.dev). The checker is in
[cpasveer/stepbench](https://github.com/cpasveer/stepbench).

Version 1: levels 1 to 3 (Fits, Printable, Strong) on three setups (facing,
flange, pipe), one task each (seed 1); four models (Claude Fable 5.2,
Opus 5.5, Sonnet 5.5, Haiku 5.5) in three CAD programs (build123d, FreeCAD,
Rhino 9). Runs are added as they finish.

## What is here

| folder or file | what |
|---|---|
| `tasks/`, `tasks-freecad/`, `tasks-rhino/` | the nine tasks as the agent received them, one copy per program (the starter script differs) |
| `MANIFEST.json`, `check_manifest.py` | the SHA-256 fingerprint of every task file; each run checked its task against it before starting |
| `AGENT.md`, `AGENT-freecad.md` | the agent's whole instructions; `TASK_FOLDER` and `WHO` are filled in per run |
| `COORDINATOR.md` | how each run was carried out: a fresh cloud machine, the model under test started as a subagent with exactly that prompt, no help |
| `runs/<program>/<model>/<task>/` | one run: the final `submission.py` and `output.step`, every graded attempt in `attempts/` with the checker's full report, `attempts.jsonl`, the agent's scratch files, `run.json` (times, checks, outcome) and `usage.json` (tokens per message) |

## How a run is judged

The agent may run the checker at most six times. A run passes when its last
check meets every requirement of its level. Checker: `9c2a54c`, frozen for
version 1 before the runs started.

Prices are computed from `usage.json` (input, output, cache-write and
cache-read tokens per message) at the provider's published list prices on
16 November 2026; nothing is estimated.

## Licence

Tasks and results: Creative Commons Attribution 4.0 (CC BY 4.0),
https://creativecommons.org/licenses/by/4.0/. The checker's code is under
the Apache License 2.0, in its own repository.
