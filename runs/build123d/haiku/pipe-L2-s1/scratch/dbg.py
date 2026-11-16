import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
part = S.build()
for s in part.solids():
    print("solid vol", round(s.volume,1), "centre", s.center())
