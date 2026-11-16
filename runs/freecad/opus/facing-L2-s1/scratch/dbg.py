exec(open('submission.py').read().split('# freecadcmd exits 0')[0].replace('return placed(body)','return body'))
b=build(); print(b.BoundBox); print(to_local(FACES[1]["origin"]))
for bo in BOLTS: print(bo["id"], to_local(bo["position"]))
print(len(b.Faces), b.isValid(), b.Volume)
