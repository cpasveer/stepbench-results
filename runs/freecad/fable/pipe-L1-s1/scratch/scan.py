import json, math, Part, FreeCAD as App
V=App.Vector
T=json.load(open("task.json")); P=T["poses"][0]
def frame(n):
    pl=P["placements"][n]; r,o=pl["rotation"],pl["origin"]
    return App.Matrix(r[0][0],r[0][1],r[0][2],o[0],r[1][0],r[1][1],r[1][2],o[1],r[2][0],r[2][1],r[2][2],o[2],0,0,0,1)
MA,MB=frame("A"),frame("B")
B=[b for b in P["resolved"]["bolts"] if b["size"]=="M4"]
for R in (6.5,7.0,7.5):
  for fx in (-1,1,3,5):
    for fy in (3,4.5,6,7.5):
        foot=MB.multVec(V(fx,fy,0)); land=MA.multVec(V(0,0,34)); d=land-foot; L=d.Length; d.normalize()
        st=Part.makeCylinder(R,L+24,foot-d*12,d)
        hs=[];ks=[]
        for b in B:
            p,ax=V(*b["position"]),V(*b["axis"])
            hs.append(st.distToShape(Part.makeCylinder(3.5,5,p+ax*6,ax))[0])
            ks.append(st.distToShape(Part.makeCylinder(2.0,26,p+ax*10,ax))[0])
        print(R,fx,fy,"head %.2f key %.2f"%(min(hs),min(ks)))
