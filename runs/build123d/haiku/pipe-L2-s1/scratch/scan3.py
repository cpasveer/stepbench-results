import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
import itertools
exec(open(str(pathlib.Path(__file__).parent/"scan2.py")).read().split("grid = []")[0])
for th, s_, r_, dt in itertools.product([-45, 45], [-5, 0, 8], [38.0], [0.0, -6.0, 6.0]):
    S.WEB_PAD_THETA = th; S.WEB_PAD_S = s_; S.WEB_PAD_R = r_; S.WEB_PLATE_DT = dt
    try:
        part = S.build(); res = evaluate(part)
    except Exception as e:
        res = {"error": str(e)[:80]}
    print(th, s_, r_, dt, res, flush=True)
