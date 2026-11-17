import json, os, FreeCAD as App, Part
V=App.Vector
T=json.load(open("task.json")); ra=T["poses"][0]["placements"]["A"]["rotation"]; D=V(ra[0][0],ra[1][0],ra[2][0])
for f in os.environ["STEPS"].split():
    s=Part.read(f); sh=0
    for fc in s.Faces:
        u0,u1,v0,v1=fc.ParameterRange; n=fc.normalAt((u0+u1)/2,(v0+v1)/2)
        sh+=fc.Area*(0.8 if abs(n.dot(D))>0.99 else 1.2)
    sh*=0.989
    print(f,"V %.1f shell %.2f score %.2f"%(s.Volume/1000,sh/1000,(0.8*sh+0.2*s.Volume)/1000), "valid",s.isValid(),len(s.Solids),len(s.Faces))
