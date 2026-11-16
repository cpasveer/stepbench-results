import json,FreeCAD as App
V=App.Vector
T=json.load(open('task.json'))['poses'][0]['resolved']['mounting_faces']
OA=V(*T[0]['origin']);nA=V(*T[0]['normal']);tA=V(-nA.y,nA.x,0)
OB=V(*T[1]['origin']);nB=V(*T[1]['normal']);tB=V(-nB.y,nB.x,0)
print(tB.dot(tA))
for s in (-5,0,5):
    P=OB+tB*s
    d=P-OA
    print(s,'n_A',d.dot(nA),'tA',d.dot(tA))
