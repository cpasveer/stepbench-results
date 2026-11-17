import runpy,sys
g=runpy.run_path('submission.py',run_name='dbg')
for x in g['sol']: print(round(x.volume,1),x.center())
p=g['part'];print(type(p),p.volume,len(p.solids()))
