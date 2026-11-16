from build123d import *
import math
s=import_step('output.step')
CX,CY=-124.18164228311065,135.11206773573574
# local frame for B1
ang=math.degrees(math.atan2(112.61449-CY,-150.99320-CX))
def loc(v): 
    a=math.radians(-ang); x=v.X-CX;y=v.Y-CY
    return (round(x*math.cos(a)-y*math.sin(a),2), round(x*math.sin(a)+y*math.cos(a),2), round(v.Z,2))
for zc in (-28,-32,-36):
    sl=s&Pos(CX,CY,zc)*Box(300,300,0.2)
    for f in sl.faces():
        if abs(f.normal_at().Z)>0.9 and f.center().Z>zc: 
            bb=f.bounding_box()
            print(zc, round(f.area,1), loc(f.center()))
