# Coordinator of one STEP.bench v1 run

You coordinate ONE run. You never design anything and never help the agent under test.
The session's prompt gives you MODEL (fable, opus, sonnet or haiku) and TASK (for example `facing-L2-s0`).
FreeCAD runs: if the prompt also gives TOOL=freecad, use `AGENT-freecad.md` instead of `AGENT.md`, `sh setup-freecad.sh` instead of `sh setup.sh` (it must print "gmsh ok" and "freecad ok"), and `tasks-freecad/TASK` instead of `tasks/TASK` everywhere below (manifest check, TASK_FOLDER, run.json, usage.json, commit), and push to the branch `run/MODEL-freecad/TASK`; without TOOL, or with TOOL=build123d, everything stays as written and the branch is `run/MODEL/TASK`. run.json always records `"tool": "freecad"` or `"tool": "build123d"`; for FreeCAD also `"freecad_version"`, as setup-freecad.sh prints it on its "freecad ok" line.

1. Setup, from the repository root: `sh setup.sh`. It must print "gmsh ok". If not, stop and report the exact error.
   Check the frozen task: `python3 check_manifest.py tasks/TASK` must print "manifest ok". If not, stop and report.
2. Create and switch to the branch `run/MODEL/TASK`.
3. Make the agent's prompt from `AGENT.md`: replace TASK_FOLDER with `tasks/TASK` and WHO with MODEL. Change nothing else.
4. Start ONE subagent with the Agent tool, `model: "MODEL"`, with exactly that prompt and nothing more.
   Wait for it to finish without polling in tight loops. If it asks anything, reply only "Follow AGENT.md."
5. Write `tasks/TASK/run.json`:
   `{"round": "v1", "checker": <contents of kit/REVISION>, "model": MODEL, "task": TASK, "start_utc", "end_utc",
   "wall_clock_s", "tokens": <the subagent's total tokens>, "tool_uses", "checks": <lines in attempts.jsonl>,
   "last_outcome", "last_failing": [...], "stiffness_last": <the STIFFNESS line of the last report>,
   "per_check": [{"n", "finished_utc", "approx_check_s"}], "setup_s", "nproc", "python"}`.
5b. Token usage for prices: the Agent tool's result names the subagent's transcript file (`output_file`, a JSON-lines file). With a small script, read every assistant message's `usage` from it, once per message id (a message can appear more than once), and write `tasks/TASK/usage.json`: a list with, per message, `input_tokens`, `cache_creation_input_tokens` (and, if present, its split `ephemeral_5m_input_tokens` / `ephemeral_1h_input_tokens`), `cache_read_input_tokens` and `output_tokens`. Add the four totals to run.json as `usage_totals`. Do not compute prices; the owner's site does that from these counts.
6. Commit everything in `tasks/TASK` (submission.py, the part files, attempts/, attempts.jsonl, scratch/, run.json, usage.json)
   and push the branch. Change nothing outside `tasks/TASK`.
7. Reply in two lines: pass or fail with the number of checks, and the tokens and total time.

Rules: push only your own branch; no other repository; nothing installed beyond setup.sh; never kill processes by name.
