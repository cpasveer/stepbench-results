src=open('submission.py').read().split("OUT = os.path.join")[0]
exec(src)
s=Part.read('output.step')
for i,f in enumerate(s.Faces):
    if abs(f.BoundBox.ZLength)<1e-6 and f.BoundBox.ZMax>28:
        print(i,[ (round(to_a(v.Point)[0],2),round(to_a(v.Point)[1],2)) for v in f.Vertexes])
