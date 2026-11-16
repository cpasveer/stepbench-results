import json, math, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); P=T["poses"][0]
def frame(n):
    pl=P["placements"][n]; r,o=pl["rotation"],pl["origin"]
    return App.Matrix(r[0][0],r[0][1],r[0][2],o[0],r[1][0],r[1][1],r[1][2],o[1],r[2][0],r[2][1],r[2][2],o[2],0,0,0,1)
MA,MB=frame("A"),frame("B")
BB=P["resolved"]["bolts"]
res=[]
R=7.0
for phi in (0,-6,-10,-14):
 for axl in (-8,0,8):
  for fx in (-2,0,2,4):
    for fy in (0,2,4,6):
        foot=MB.multVec(V(fx,fy,0)); a=math.radians(phi)
        land=MA.multVec(V(axl,34*math.sin(a),34*math.cos(a))); d=land-foot; L=d.Length; d.normalize()
        st=Part.makeCylinder(R,L+24,foot-d*12,d)
        hs=[];ks=[];sp=[]
        for b in BB:
            p,ax=V(*b["position"]),V(*b["axis"])
            if b["size"]=="M4":
                hs.append(st.distToShape(Part.makeCylinder(3.5,5,p+ax*6,ax))[0])
                ks.append(st.distToShape(Part.makeCylinder(2.0,26,p+ax*10,ax))[0])
            else:
                sp.append(st.distToShape(Part.makeCylinder(6,30,p+ax*5.5,ax))[0])
        res.append((min(min(hs),min(ks)),phi,axl,fx,fy,min(hs),min(ks),min(sp)))
res.sort(reverse=True)
for r in res[:15]: print("RES",[round(x,2) for x in r])
