import sys, pathlib, itertools
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
exec(open(str(pathlib.Path(__file__).parent/"scan2.py")).read().split("grid = []")[0])
for th, s_, r_, w_ in itertools.product([15, 20, 25], [-23, -25], [36.0, 38.0], [(10.0, 8.0)]):
    S.WEB_PAD_THETA = th; S.WEB_PAD_S = s_; S.WEB_PAD_R = r_; S.WEB_PLATE_DT = 0.0
    S.WEB_PAD_W, S.WEB_PAD_H = w_
    try:
        part = S.build(); res = evaluate(part)
    except Exception as e:
        res = {"error": str(e)[:80]}
    print(th, s_, r_, w_, res, flush=True)
