from build123d import *
import build123d, inspect
print(build123d.__version__ if hasattr(build123d,'__version__') else '')
print([n for n in dir(Solid) if 'extrude' in n or 'cylinder' in n])
s = import_step('fixed_A.step'); print(type(s), s.bounding_box())
t = import_step('fixed_B.step'); print(type(t), t.bounding_box())
