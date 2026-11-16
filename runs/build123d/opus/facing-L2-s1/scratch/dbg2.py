import sys; sys.path.insert(0,'.')
import submission as S
from build123d import *
k=S.kite(); print(k.bounding_box())
t=extrude(k,10); print(t.bounding_box())
a=S.a_flange(); print(a.bounding_box())
