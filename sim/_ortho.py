import trimesh,glob,os,sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
d='Arm_Bot_STLs/Arm_Bot_STLs'
names=sys.argv[2:]
fig,axs=plt.subplots(len(names),3,figsize=(18,5*len(names)))
axs=axs.reshape(len(names),3)
for i,n in enumerate(names):
    m=trimesh.load(f'{d}/{n}.STL')
    v=m.vertices
    for j,(a,b,t) in enumerate([(0,1,'XY'),(0,2,'XZ'),(1,2,'YZ')]):
        ax=axs[i,j]
        ax.triplot(v[:,a],v[:,b],m.faces,lw=.15,color='#468') if False else None
        # silhouette via face polygons
        from matplotlib.collections import PolyCollection
        pc=PolyCollection(v[m.faces][:,:,[a,b]],facecolor='#9bb',edgecolor='#456',linewidths=.1,alpha=.5)
        ax.add_collection(pc);ax.autoscale();ax.set_aspect('equal');ax.set_title(f'{n} {t}');ax.grid(True,lw=.3)
plt.tight_layout();plt.savefig(sys.argv[1],dpi=55)
