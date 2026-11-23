import sys,numpy as np,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
n=sys.argv[1];d=np.load(f'results/{n}_dump.npz');g=d['grid'];p=d['path'];h=np.load(f'results/{n}_traj.npy');gt=np.load('gt.npy')
O=-1.5;R=.005;x0,y0=0.33,0.33
fig,ax=plt.subplots(figsize=(8,8))
ax.imshow(g,origin='lower',extent=[O,O+g.shape[1]*R,O,O+g.shape[0]*R],cmap='gray_r',vmin=-1,vmax=1)
ax.plot(gt[:,0]-x0,gt[:,1]-y0,'c-',lw=.6);ax.plot(h[:,1]-x0,h[:,2]-y0,'r.',ms=1)
if len(p):ax.plot(p[:,0],p[:,1],'g.-',ms=2)
cx,cy=h[-1,1]-x0,h[-1,2]-y0;w=float(sys.argv[2]) if len(sys.argv)>2 else .4
ax.set_xlim(cx-w,cx+w);ax.set_ylim(cy-w,cy+w);fig.savefig(f'results/{n}_dbg.png',dpi=90)
