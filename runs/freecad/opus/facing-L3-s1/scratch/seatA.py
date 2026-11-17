import FreeCAD as App, Part, json
V=App.Vector
s=Part.read('output.step'); T=json.load(open('task.json')); R=T['poses'][0]['placements']['A']['rotation']
LX=V(R[0][0],R[1][0],0); LY=V(R[0][1],R[1][1],0); c=V(-126,127.5,0)
for f in s.Faces:
    if abs(f.CenterOfMass.z+94)<1e-6:
        print('A seat max', max(max(abs((v.Point-c).dot(LX)),abs((v.Point-c).dot(LY))) for v in f.Vertexes), 'area', f.Area)
