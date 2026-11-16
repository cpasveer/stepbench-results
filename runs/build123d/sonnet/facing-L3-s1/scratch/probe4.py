from build123d import *
import math
s=import_step('output.step')
CX,CY=-124.18164228311065,135.11206773573574
ang=math.atan2(112.61449-CY,-150.99320-CX)
def pt(x,y,z):
    return Vector(CX+x*math.cos(ang)-y*math.sin(ang), CY+x*math.sin(ang)+y*math.cos(ang), z)
for x in (35,30,25):
  print('x',x)
  for z in (-26,-28,-29.5,-30.5,-32,-35,-38,-42):
    print(z, ''.join('#' if s.is_inside(pt(x,y,z)) else '.' for y in [i*0.5 for i in range(-20,21)]))
