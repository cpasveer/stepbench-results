from build123d import import_step
import numpy as np
o=import_step('output.step');a=import_step('fixed_A.step');b=import_step('fixed_B.step')
print(o.bounding_box().min,o.bounding_box().max)
print(a.bounding_box().min,a.bounding_box().max)
print(o.distance_to(a),o.distance_to(b))
from build123d import Plane
