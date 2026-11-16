src=open('submission.py').read().split('# freecadcmd exits 0')[0]
src=src.replace('    body = body.makeFillet(FILLET, sel)','    raise StopIteration(sel, body)')
exec(src)
try: build()
except StopIteration as ex:
    sel, body = ex.args
import sys
for e in sel:
    c=e.CenterOfMass
    try: body.makeFillet(4.0,[e]); ok=True
    except Exception: ok=False
    sys.stderr.write("%s %.1f %.1f %.1f L=%.1f %s\n"%(ok,c.x,c.y,c.z,e.Length,type(e.Curve).__name__))
