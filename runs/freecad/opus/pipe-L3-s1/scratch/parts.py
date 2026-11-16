import sys
sys.argv=['submission']
exec(open('submission.py').read().split("OUT = os.path")[0])
sa=saddle(); pl=plate(); w=web()
print('saddle',sa.Volume,'plate',pl.Volume,'web',w.Volume,'web-saddle',w.cut(sa).Volume, 'web-plate-saddle', w.cut(sa).cut(pl).Volume)
