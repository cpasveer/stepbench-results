# rough printed-material estimate: shell 1.2 on sides, 0.8 on top/bottom, 20 % infill
import Part, os
s=Part.read(os.environ.get("STEP","output.step"))
sh=0
for f in s.Faces:
    try:
        u0,u1,v0,v1=f.ParameterRange; n=f.normalAt((u0+u1)/2,(v0+v1)/2)
        t=0.8 if abs(n.z)>0.99 else 1.2
    except Exception: t=1.2
    sh+=f.Area*t
sh=min(sh,s.Volume)
print("MAT vol %.1f shell~%.1f infill~%.1f total~%.1f cm3"%(s.Volume/1000,sh/1000,0.2*(s.Volume-sh)/1000,(sh+0.2*(s.Volume-sh))/1000))
