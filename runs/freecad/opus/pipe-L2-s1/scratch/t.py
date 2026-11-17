import Part, FreeCAD as App
print([k for k in dir(Part) if 'ipe' in k or 'BRep' in k])
w=Part.Wire(Part.LineSegment(App.Vector(0,0,0),App.Vector(0,0,10)).toShape())
ps=Part.BRepOffsetAPI.MakePipeShell(w)
print([m for m in dir(ps) if not m.startswith('_')])
