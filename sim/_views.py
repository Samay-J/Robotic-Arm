import trimesh,glob,os
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
d='Arm_Bot_STLs/Arm_Bot_STLs'
fs=sorted(glob.glob(d+'/*.STL'))
fig=plt.figure(figsize=(20,14))
for i,f in enumerate(fs):
    m=trimesh.load(f)
    for j,(el,az) in enumerate([(25,-60)]):
        ax=fig.add_subplot(3,4,i+1,projection='3d')
        tri=m.vertices[m.faces]
        ax.add_collection3d(Poly3DCollection(tri,alpha=.9,facecolor='#9bb',edgecolor='none'))
        b=m.bounds; c=(b[0]+b[1])/2; r=max(m.extents)/2
        ax.set_xlim(c[0]-r,c[0]+r);ax.set_ylim(c[1]-r,c[1]+r);ax.set_zlim(c[2]-r,c[2]+r)
        ax.view_init(el,az);ax.set_title(os.path.basename(f)+' '+str([round(x) for x in m.extents]))
        ax.set_xlabel('X');ax.set_ylabel('Y');ax.set_zlabel('Z')
plt.savefig('sim/parts.png',dpi=60)
