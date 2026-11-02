#!/usr/bin/env python3
"""Engineering scene overview, explicitly not an RViz capture."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from grasp01_core import SceneGeometry,BOTTLE


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('config','installation','description','backend','srdf','out'):
        p.add_argument('--'+k,required=True,type=Path)
    a=p.parse_args();g=SceneGeometry(json.loads(a.config.read_text()),json.loads(a.installation.read_text()),a.description,a.backend,a.srdf)
    try:
        q=np.r_[np.deg2rad(g.config['initial_urdf_deg']),g.open_q,g.open_q]
        ts=g.robot.fk(q);ts['vision_assembly']=ts['Empty_Link6']@g.camera_local
        objects=[]
        for n,t in {**g.world,**ts}.items():
            if n not in g.meshes:continue
            mesh=g.meshes[n].moved(t)
            color='#d4593b' if n=='base_link' else ('#2775ce' if n==BOTTLE else ('#b0b0b0' if n=='desk' else ('#dd8d29' if n=='vision_assembly' else ('#42a89a' if n.startswith('ganten') else '#78859a'))))
            objects.append((n,mesh,color))
        fig=plt.figure(figsize=(12,9));ax=fig.add_subplot(111,projection='3d')
        for n,m,c in objects:
            poly=Poly3DCollection(m.vertices[m.faces],facecolor=c,edgecolor='none',alpha=.22 if n=='desk' else 1.)
            ax.add_collection3d(poly)
        allv=np.vstack([m.vertices for _,m,_ in objects]);lo=allv.min(0);hi=allv.max(0)
        ax.set(xlim=(lo[0]-.05,hi[0]+.05),ylim=(lo[1]-.05,hi[1]+.05),zlim=(-.04,max(.65,hi[2]+.04)),xlabel='base X (m)',ylabel='base Y (m)',zlabel='base Z (m)')
        ax.set_box_aspect([hi[0]-lo[0],hi[1]-lo[1],.69]);ax.view_init(25,35)
        ax.set_title('GRASP-01 / DRAFT_SIMULATION / offline mesh view\n'+g.config['label']+'; fixed base intersects desk padding; no motion',fontsize=13)
        fig.text(.05,.035,'Table: VISION-02 input | Rack/bottle: MODEL-01 | Orange: moving camera envelope\nRed base: padding conflict, not evidence of physical table penetration. NOT an RViz screenshot.',fontsize=10)
        a.out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.out,dpi=160);plt.close(fig)
        print(a.out)
    finally:g.close()

if __name__=='__main__':main()
