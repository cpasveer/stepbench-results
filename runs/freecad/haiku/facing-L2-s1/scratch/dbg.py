import math, json
import FreeCAD as App, Part
V = App.Vector
PAD_R=4.25; TAB_W=4.0; R_BODY=28.0; zB=-25.5; z_top_bot=-33.5
p=(-150.99319858537234,112.61449077750642)
bx,by=-124.18164228311065,135.11206773573574
k_cone=1.2; k_tab=1.0; d_ext=1.0
boss = Part.makeCylinder(PAD_R, zB - z_top_bot + d_ext, V(p[0], p[1], z_top_bot - d_ext), V(0, 0, 1))
print("boss valid", boss.isValid())
cone = Part.makeCone(PAD_R + d_ext / k_cone, 0.0, k_cone * PAD_R + d_ext, V(p[0], p[1], z_top_bot - d_ext), V(0, 0, 1))
print("cone valid", cone.isValid())
b2 = boss.cut(cone)
print("boss cut valid", b2.isValid(), len(b2.Solids), round(b2.Volume,2))
dx,dy=p[0]-bx,p[1]-by; L=math.hypot(dx,dy); phi=math.degrees(math.atan2(dy,dx)); s_rim=L-PAD_R
pts=[V(0,0,zB),V(L,0,zB),V(L,0,z_top_bot+k_tab*PAD_R),V(s_rim,0,z_top_bot),V(R_BODY,0,z_top_bot+k_tab*(s_rim-R_BODY)),V(0,0,z_top_bot+k_tab*(s_rim-R_BODY)),V(0,0,zB)]
f=Part.Face(Part.makePolygon(pts)); print("face valid", f.isValid(), f.Area)
t=f.extrude(V(0,TAB_W,0)); print("tab valid", t.isValid(), round(t.Volume,2))
