import numpy as np
R=np.array([[0.2487956429525063,-0.9470984120208662,0.20274448450057206],[0.5900653629615055,-0.017782176257166975,-0.8071596258737586],[0.7680648380709507,0.32045029591983953,0.5544258402743613]])
D=R[:,0];NB=R[:,2];TG=R[:,1]
n=np.array([0.3885173161164798,0.9214414224885092,0])
C=np.array([19.5496,49.6857,52.2473]);O=np.array([21,136.5,-7])
L=O+34*NB
bd=(L-C);bd/=np.linalg.norm(bd)
def rad(th):return np.cos(np.radians(th))*NB+np.sin(np.radians(th))*TG
best=None
rng=np.random.default_rng(0)
for w in rng.normal(size=(200000,3)):
    w/=np.linalg.norm(w)
    cons=[-n@w]  # B rear face normal -n : need >=-0.707 (want measure)
    m=min([ -(-n)@w*-1 for _ in [0]])
    # margins: normal.w >= -0.7071
    vals=[(-n)@w]
    for th in np.linspace(-48,48,25):
        vals.append(rad(th)@w); vals.append(-rad(th)@w)
    vals.append(-bd@w*0+ (-(bd@w)) if False else 0)
    # beam axis: sides ok if |axis.w|>=0.707
    vals.append(abs(bd@w)-0.0+(-0.7071+0.7071))  # placeholder
    mg=min(vals[:-2])+0.7071
    mb=abs(bd@w)-0.7071
    m=min(mg,mb)
    if best is None or m>best[0]:best=(m,w.copy(),mg,mb)
print(best)
print('----')
a1=R[:,1]*0  # placeholder
best=None
rng=np.random.default_rng(3)
a1v=np.array([0.7093118905965579,-0.6509870200868549,0.27035632165831364]);a3v=np.array([-0.3771547728198181,-0.6713858946506742,0.6379617989560074])
ths=np.linspace(-44,44,23)
for w in rng.normal(size=(300000,3)):
    w/=np.linalg.norm(w)
    vals=[-abs(n@w)]  # need |n.w|<=.707 -> margin .707-|nw|
    m=[0.7071-abs(n@w), abs(bd@w)-0.7071]
    for th in ths:
        r=rad(th)
        m.append(0.7071+r@w); m.append(0.7071-r@w)
    for a in (a1v,a3v): m.append(0.7071+a@w)
    mm=min(m)
    if best is None or mm>best[0]:best=(mm,w.copy(),m[0],m[1])
print(best, 'beam angle from w',np.degrees(np.arccos(abs(bd@best[1]))))
