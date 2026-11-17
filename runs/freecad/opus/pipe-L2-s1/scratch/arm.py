exec(open('submission.py').read().split("OUT = os.path")[0])
a=arm()
A=Part.read('fixed_A.step')
print('arm valid',a.isValid(),a.Volume, 'common A', a.common(A).Volume)
for f in a.Faces: print(type(f.Surface).__name__, round(f.Area,1))
# end face
import numpy as np
for f in a.Faces:
    if 'Plane' in type(f.Surface).__name__:
        for v in f.Vertexes:
            q=v.Point-OA; print(' vtx x',round(q.dot(EX),2),'y',round(q.dot(EY),2),'z',round(q.dot(EZ),2))
