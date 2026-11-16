from build123d import import_step, GeomType
p=import_step('output.step')
for f in p.faces():
    if f.geom_type==GeomType.PLANE:
        n=f.normal_at(f.center())
        if n.Z<-0.5:
            c=f.center(); print(round(f.area,2), round(n.Z,3), round(c.X,2),round(c.Y,2),round(c.Z,2))
