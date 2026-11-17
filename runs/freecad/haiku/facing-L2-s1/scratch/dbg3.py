import math
import FreeCAD as App, Part
V = App.Vector
PAD_R=4.25; TAB_W=4.0; R_BODY=28.0; zB=-25.5; z_top_bot=-33.5
B=[(-150.99319858537234,112.61449077750642),(-130.25933730110913,169.58034508725697),(-91.29239096285048,123.14136734244381)]
bx,by=-124.18164228311065,135.11206773573574
def make(boss_on, tab_on, k_cone=1.2, k_tab=1.0):
    base = Part.makeCylinder(R_BODY, zB-(-94+8)+1, V(bx,by,-94+8-1), V(0,0,1))
    sh = base
    for p in B:
        if boss_on:
            d=1.0
            boss = Part.makeCylinder(PAD_R, zB-z_top_bot+d, V(p[0],p[1],z_top_bot-d), V(0,0,1))
            cone = Part.makeCone(PAD_R+d/k_cone, 0.0, k_cone*PAD_R+d, V(p[0],p[1],z_top_bot-d), V(0,0,1))
            boss = boss.cut(cone)
            sh = sh.fuse(boss)
        if tab_on:
            dx,dy=p[0]-bx,p[1]-by; L=math.hypot(dx,dy); phi=math.degrees(math.atan2(dy,dx)); s_rim=L-PAD_R
            pts=[V(0,0,zB),V(L,0,zB),V(L,0,z_top_bot+k_tab*PAD_R),V(s_rim,0,z_top_bot),V(R_BODY,0,z_top_bot+k_tab*(s_rim-R_BODY)),V(0,0,z_top_bot+k_tab*(s_rim-R_BODY)),V(0,0,zB)]
            t=Part.Face(Part.makePolygon(pts)).extrude(V(0,TAB_W,0)); t.translate(V(0,-TAB_W/2,0)); t.rotate(V(0,0,0),V(0,0,1),phi); t.translate(V(bx,by,0))
            sh = sh.fuse(t)
    return sh
for name,(bo,ta) in {"body only":(False,False),"bosses":(True,False),"tabs":(False,True),"both":(True,True)}.items():
    s = make(bo,ta).removeSplitter()
    s.exportStep("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/scratch/v.step")
    r = Part.read("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/scratch/v.step")
    print("VARIANT", name, "mem", s.isValid(), "roundtrip", r.isValid())
