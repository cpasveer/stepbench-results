You are being tested as a CAD designer. Your whole task is in one folder of this repository:

TASK_FOLDER

Read README.md there first; it is the brief. task.json holds every number. Model what the brief asks for and save it in that folder under the file names the brief gives, using build123d.

Setting up (once, from the repository root):

    sh setup.sh

How to run things (from inside the task folder):
- Python with build123d:   uv run --project ../../agent-env python submission.py
- The checker:             JOINTBENCH_MAX_CHECKS=6 uv run --project ../../kit jointbench verify . --by WHO --tool build123d

Rules, all binding:
- Work only inside the task folder. Put any scratch files in a `scratch/` folder inside it. Do not read or list anything outside it: not the other task folders, and nothing of the jointbench source code in kit/. You may run the commands above and import build123d. Run your own Python only in agent-env; the kit environment is for the checker command alone.
- No web access, apart from the setup step above.
- You may run the checker at most six times in total. Write fresh output files before each one. A check can take several minutes; wait for it.
- Never kill processes by name.
- Keep your final modelling script as submission.py in the task folder, so that running it reproduces your answer.

The brief lists every requirement; beyond them, make the best design you can, as an experienced mechanical designer would.

When you are done:
1. Commit everything in the task folder (your files, attempts/, attempts.jsonl, scratch/) to this session's branch and push it.
2. Reply in at most five sentences: whether the last check passed, how many times you ran the checker, and what you made and why.
