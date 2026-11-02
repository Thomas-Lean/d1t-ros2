#!/usr/bin/env python3
"""No ROS clients: full initial-arm geometry and bounded MODEL local diagnostics."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from grasp01_core import SceneGeometry


def run(config, installation, description, backend, srdf, output):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    scene=SceneGeometry(config,installation,description,backend,srdf)
    try:
        q=np.r_[np.deg2rad(config['initial_urdf_deg']),scene.open_q,scene.open_q]
        initial=scene.check(q)
        with gzip.open(output/'initial_full_pairs.json.gz','wt') as f:
            json.dump(initial,f)
        local=scene.a.scenario(scene.model,description,backend,output/'local',label=config['label'],fixed_axis=scene.axis)
        # Refine the first local approach obstruction at 0.1 mm; no motion.
        rows=local['sampling']['all_sample_summaries']
        approach=[r for r in rows if r['phase']=='approach']
        first=next((i for i,r in enumerate(approach) if any(r[g]['blocked_pairs'] for g in ('hand_rack','hand_bottle','bottle_rack'))),None)
        refined=[]
        if first is not None and first>0:
            engine=scene.a.Proximity(backend,{**scene.rack,**scene.links,'ganten_348_bottle':scene.bottle})
            try:
                for remaining in scene.a.inclusive(approach[first-1]['value_mm'],approach[first]['value_mm'],.1):
                    wrist=scene.g.pose(translation=-scene.wrist[:3,0]*remaining/1000)@scene.wrist
                    pairs=scene.a.inspect_state(engine,scene.model,scene.rack,scene.links,scene.joints,scene.metrics,wrist,scene.bt,scene.model['draft']['open_gap'],'approach',scene.bt)
                    refined.append({'remaining_approach_mm':float(remaining),'pairs':pairs})
            finally:
                engine.close()
        with gzip.open(output/'refined_pairs.json.gz','wt') as f:
            json.dump(refined,f)
        result={'scene':scene.metadata(),'initial_urdf_rad':q.tolist(),
                'initial_full_arm_blocked':initial['blocked'],
                'initial_bounds':initial['bounds_violations'],
                'initial_blocking_pairs':[r for r in initial['pairs'] if r['blocked']],
                'local_phases':local['phases'],
                'refined_first_block':next(({'remaining_approach_mm':r['remaining_approach_mm'],'blocking_pairs':[p for p in r['pairs'] if p['blocked']]} for r in refined if any(p['blocked'] for p in r['pairs'])),None),
                'boundary':'Offline initial whole arm plus local hand path. No MoveIt plan or fake execution; local post-grasp stages conditional; no continuous collision proof.'}
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({'label':config['label'],'initial_blocked':initial['blocked'],'pairs':[r['pair'] for r in initial['pairs'] if r['blocked']], 'local_approach_blocked':local['phases']['approach']['blocked_samples'],'refined_first_remaining_mm':result['refined_first_block']['remaining_approach_mm'] if result['refined_first_block'] else None}),flush=True)
        return result
    finally:
        scene.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('config','installation','description','backend','srdf','out'):
        p.add_argument('--'+k,required=True,type=Path)
    a=p.parse_args()
    run(json.loads(a.config.read_text()),json.loads(a.installation.read_text()),a.description,a.backend,a.srdf,a.out)

if __name__=='__main__':main()
