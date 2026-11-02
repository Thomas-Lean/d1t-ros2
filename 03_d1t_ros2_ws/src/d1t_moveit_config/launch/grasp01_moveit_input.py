#!/usr/bin/env python3
"""Generate MODEL/VISION-derived input for no-node MoveIt collision verification."""
import argparse
import json
from pathlib import Path
import numpy as np
from grasp01_core import SceneGeometry,JOINTS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('config','installation','description','backend','srdf','out'):
        p.add_argument('--'+k,required=True,type=Path)
    a=p.parse_args()
    g=SceneGeometry(json.loads(a.config.read_text()),json.loads(a.installation.read_text()),a.description,a.backend,a.srdf)
    try:
        world=[]
        for n,t in g.world.items():
            if n=='desk':world.append({'id':n,'matrix':t.tolist(),'size':g.desk_spec['size_m']})
            else:world.append({'id':n,'matrix':t.tolist(),'vertices':g.meshes[n].vertices.tolist(),'faces':g.meshes[n].faces.tolist()})
        camera={'id':'vision_assembly','frame':'Empty_Link6','size':g.camera_spec['size_m'],'matrix':g.camera_local.tolist(),'touch_links':['Empty_Link6']}
        q=np.r_[np.deg2rad(g.config['initial_urdf_deg']),g.open_q,g.open_q]
        states=[{'label':'initial','joints':dict(zip(JOINTS+['Joint_L','Joint_R'],q))}]
        data={'urdf':str((a.description/'urdf/d1t.urdf').resolve()),'srdf':str(a.srdf.resolve()),'world':world,'attached':[camera],'states':states}
        a.out.write_text(json.dumps(data,separators=(',',':'))+'\n')
        print(json.dumps({'input':str(a.out),'world_ids':[o['id'] for o in world],'attached_ids':['vision_assembly'],'state_count':len(states)}))
    finally:g.close()

if __name__=='__main__':main()
