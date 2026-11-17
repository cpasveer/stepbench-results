import math, os, Part
for f in os.environ.get("STEPS","output.step").split():
    s=Part.read(f)
    print(f,"min edge %.3f"%min(e.Length for e in s.Edges),"min face area %.2f"%min(x.Area for x in s.Faces))
    worst=180
    for fi,fc in enumerate(s.Faces):
        for w in fc.Wires:
            es=w.OrderedEdges
            for i,e in enumerate(es):
                e2=es[(i+1)%len(es)]
                if len(es)==1 or e.isClosed() or e2.isClosed(): continue
                # common vertex
                for v in e.Vertexes:
                    for v2 in e2.Vertexes:
                        if v.Point.distanceToPoint(v2.Point)<1e-6:
                            def tang(ed,p):
                                a,b=ed.FirstParameter,ed.LastParameter
                                pa=ed.valueAt(a)
                                if pa.distanceToPoint(p)<1e-6: return ed.tangentAt(a)
                                return ed.tangentAt(b)*-1
                            t1=tang(e,v.Point); t2=tang(e2,v.Point)
                            ang=math.degrees(t1.getAngle(t2))
                            if ang<worst: worst=ang; where=(fi,tuple(round(c,2) for c in v.Point))
    print("  sharpest face corner %.2f deg"%worst, where)
