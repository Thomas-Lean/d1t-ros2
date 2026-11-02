#!/usr/bin/env python3
"""Rebuild the MODEL-01 DRAFT and run offline local clearance checks.

No arm, gripper, planner, service or action clients. Mesh distances are FCL
separations in metres; collisions have distance zero, NOT an invented depth.
"""
import argparse
import copy
import gzip
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np

from model01_geometry import (Mesh, bottle_pose, box, cylinder, export,
                              finger_transform, grasp, gripper_metrics,
                              load_config, pose, profile, profile_x, quaternion,
                              rack, urdf_gripper)


class Proximity:
    def __init__(self, executable, meshes):
        self.proc = subprocess.Popen([str(executable)], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, text=True)
        self.call({'load':{name:m.payload() for name,m in meshes.items()}})

    def call(self, request):
        self.proc.stdin.write(json.dumps(request,separators=(',',':'))+'\n')
        self.proc.stdin.flush()
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError('proximity backend exited without a result')
        result = json.loads(line)
        if isinstance(result,dict) and 'error' in result:
            raise RuntimeError(result['error'])
        return result

    def pairs(self, pairs):
        return self.call({'queries':[{'a':a,'b':b,'ta':ta.tolist(),'tb':tb.tolist()}
                                     for a,b,ta,tb in pairs]})

    def close(self):
        self.proc.stdin.close()
        self.proc.stdout.close()
        code = self.proc.wait(timeout=10)
        if code:
            raise RuntimeError(f'proximity backend exit {code}')


def inclusive(start, stop, maximum_step):
    return np.linspace(start,stop,max(1,math.ceil(abs(stop-start)/maximum_step))+1)


def transformations(wrist, links, joints, metrics, gap):
    q = (metrics['urdf_open_gap_mm']-gap)/2000
    return {name:wrist if name == 'Empty_Link6' else wrist@finger_transform(joints,name,q)
            for name in links}


def bottle_contact_allowed(result, name, wrist, gap, c, metrics):
    """Only inner finger-plane contact at the final cylinder closing gap.

    No wrist-bottle contact or finger-backstop contact is exempt. Overclosure
    is always rejected, even if an intersection happens to lie on the plane.
    """
    tol = c['sampling']['contact_tolerance']
    diameter = c['reported']['bottle_diameter']
    if name not in ['Empty_Link_L','Empty_Link_R'] or abs(gap-diameter) > 1e-7 or result['contained']:
        return False
    pts = np.asarray(result['contacts_m'])
    if not len(pts) or len(pts)>=64:  # capped contact output cannot justify an exemption
        return False
    local = (pts-wrist[:3,3])@wrist[:3,:3]*1000
    plane = metrics['inner_center_y_mm'] + (-gap/2 if name.endswith('_L') else gap/2)
    return bool(np.all(np.abs(local[:,1]-plane) <= tol))


def choose_axis_x(engine, c, bt, links, joints, tcp, metrics):
    """Find a conservative insertion bound with the actual finger and wrist meshes.

    Search only the bottle/hand relation. Rack collisions never influence this
    offset, so an obstructed rack is not 'fixed' by silently changing the grasp.
    """
    diameter = c['reported']['bottle_diameter']
    candidates = inclusive(tcp[0,3]*1000,metrics['finger_tip_x_mm']-5,0.25)
    checks = []
    valid = None
    for x in candidates:
        wrist,_ = grasp(c,bt,metrics,tcp,x)
        ts = transformations(wrist,links,joints,metrics,diameter)
        pairs = [(name,'ganten_348_bottle',t,bt) for name,t in ts.items()]
        results = engine.pairs(pairs)
        blocked = [p[0] for p,r in zip(pairs,results)
                   if r['collision'] and not bottle_contact_allowed(r,p[0],wrist,diameter,c,metrics)]
        checks.append({'axis_x_mm':float(x),'blocked_by':blocked})
        if not blocked:
            valid = float(x)+c['draft']['backstop_gap']
            break
    if valid is None:
        raise ValueError('no insertion pose in tested finger reach; no grasp assumed')
    if valid > metrics['finger_tip_x_mm']:
        raise ValueError('axis lies beyond fingertip')
    return valid, checks


def inspect_state(engine,c,obstacles,links,joints,metrics,wrist,bt,gap,phase,seated_bottle):
    ts = transformations(wrist,links,joints,metrics,gap)
    pairs, groups = [], []
    for name,t in ts.items():
        for obstacle in obstacles:
            pairs.append((name,obstacle,t,np.eye(4))); groups.append('hand_rack')
        pairs.append((name,'ganten_348_bottle',t,bt)); groups.append('hand_bottle')
    for obstacle in obstacles:
        pairs.append(('ganten_348_bottle',obstacle,bt,np.eye(4))); groups.append('bottle_rack')
    results = engine.pairs(pairs)
    records = []
    for pair,group,result in zip(pairs,groups,results):
        allowed = False
        if result['collision']:
            if group == 'hand_bottle' and phase in ['close','lift','exit']:
                allowed = bottle_contact_allowed(result,pair[0],wrist,gap,c,metrics)
            if (group == 'bottle_rack' and phase in ['approach','close','lift_start']
                    and np.allclose(bt,seated_bottle,atol=1e-10,rtol=0)):
                # The independent support LP certifies non-penetration of the
                # generated polyhedron at THIS exact pose. FCL coplanar triangle
                # contact positions can lie outside the true overlap, so those
                # positions must not decide the seat-contact exemption.
                if pair[1] in ['ganten_rack_upper_plate','ganten_rack_upper_front_edge','ganten_rack_upper_back_edge'] and not result['contained']:
                    allowed = True
        records.append({'pair':pair[:2],'group':group,'allowed_contact':allowed,
                        'blocked':bool(result['collision'] and not allowed),**result})
    return records


def state_summary(records):
    out = {}
    for group in ['hand_rack','hand_bottle','bottle_rack']:
        rows = [r for r in records if r['group']==group]
        nearest = min(rows,key=lambda r:r['distance_m'])
        out[group] = {'minimum_mm':nearest['distance_m']*1000,'nearest_pair':nearest['pair'],
                      'nearest_m':nearest.get('nearest_m',nearest['contacts_m'][:1]),
                      'blocked_pairs':[r['pair'] for r in rows if r['blocked']],
                      'allowed_contact_pairs':[r['pair'] for r in rows if r['allowed_contact']]}
    return out


def scenario(c,description,backend,out=None,label='nominal',fixed_axis=None):
    obstacles,post_notes = rack(c)
    links,joints,tcp = urdf_gripper(description)
    metrics = gripper_metrics(links,joints,tcp)
    bottle = cylinder(c['reported']['bottle_diameter']/2,c['reported']['bottle_height'],int(c['sampling']['cylinder_segments']))
    bt,contacts = bottle_pose(c)
    engine = Proximity(backend,{**obstacles,**links,'ganten_348_bottle':bottle})
    try:
        axis,offset_scan = choose_axis_x(engine,c,bt,links,joints,tcp,metrics) if fixed_axis is None else (fixed_axis,[])
        wrist,bottle_tcp = grasp(c,bt,metrics,tcp,axis)
        raw = []
        r,d,s = c['reported'],c['draft'],c['sampling']
        states = []
        # Tool +X full vector is used, including pitch/yaw/roll.
        for distance in inclusive(d['approach_distance'],0,s['path_step']):
            states.append(('approach',float(distance),pose(translation=-wrist[:3,0]*distance/1000)@wrist,bt,d['open_gap']))
        for gap in inclusive(d['open_gap'],r['bottle_diameter'],s['closing_step']):
            states.append(('close',float(gap),wrist,bt,float(gap)))
        for lift in inclusive(0,d['lift_distance'],s['path_step']):
            shift = pose(translation=[0,0,lift/1000])
            states.append(('lift',float(lift),shift@wrist,shift@bt,r['bottle_diameter']))
        for distance in inclusive(0,d['front_exit_distance'],s['path_step']):
            shift = pose(translation=[0,-distance/1000,d['lift_distance']/1000])
            states.append(('exit',float(distance),shift@wrist,shift@bt,r['bottle_diameter']))
        summaries = []
        for phase,value,wt,bpt,gap in states:
            check_phase = 'lift_start' if phase=='lift' and value==0 else phase
            # At lift start finger contact still uses the closed phase rule.
            rows = inspect_state(engine,c,obstacles,links,joints,metrics,wt,bpt,gap,
                                 'close' if check_phase=='lift_start' else check_phase,bt)
            summary = {'phase':phase,'value_mm':value,**state_summary(rows)}
            summaries.append(summary)
            raw.append({'phase':phase,'value_mm':value,'wrist_matrix_m':wt.tolist(),
                        'bottle_matrix_m':bpt.tolist(),'gap_mm':gap,'pairs':rows})
        phase_results = {}
        for phase in ['approach','close','lift','exit']:
            ss = [x for x in summaries if x['phase']==phase]
            worst = min(ss,key=lambda x:x['hand_rack']['minimum_mm'])
            phase_results[phase] = {
                'samples':len(ss),'min_hand_rack_mm':worst['hand_rack']['minimum_mm'],
                'worst_value_mm':worst['value_mm'],'nearest_pair':worst['hand_rack']['nearest_pair'],
                'witness_m':worst['hand_rack']['nearest_m'],
                'blocked_samples':sum(any(x[g]['blocked_pairs'] for g in ['hand_rack','hand_bottle','bottle_rack']) for x in ss),
                'blocked_pairs':sorted({tuple(p) for x in ss for g in ['hand_rack','hand_bottle','bottle_rack'] for p in x[g]['blocked_pairs']}),
                'clears_30mm_at_samples':all(x['hand_rack']['minimum_mm']>=30 for x in ss)}
        target = next(x for x in summaries if x['phase']=='close' and abs(x['value_mm']-r['bottle_diameter'])<1e-8)
        result = {'label':label,'status':'DRAFT','parameters':c,'axis_x_mm':axis,
                  'parameter_role':('baseline reports plus explicit DRAFT assumptions' if label=='nominal'
                                    else 'synthetic sensitivity values; NOT new measurements, even under the reported schema key'),
                  'gripper':metrics,'insertion_scan':offset_scan,'post_notes':post_notes,
                  'bottle_bottom_m':bt[:3,3].tolist(),'bottle_cap_m':(bt[:3,3]+bt[:3,2]*r['bottle_height']/1000).tolist(),
                  'grip_reference_m':(bt[:3,3]+bt[:3,2]*r['grip_h']/1000).tolist(),
                  'bottle_quaternion_xyzw':quaternion(bt),'support_points_m':contacts.tolist(),
                  'bottle_to_tcp_matrix_m':bottle_tcp.tolist(),
                  'bottle_to_tcp_quaternion_xyzw':quaternion(bottle_tcp),
                  'wrist_matrix_m':wrist.tolist(),'bottle_matrix_m':bt.tolist(),
                  'target':target,'phases':phase_results,
                  'conditional_exit': 'exit is a separate conditional segment; it does not repair a blocked approach/close/lift',
                  'sampling':{'translation_max_step_mm':s['path_step'],'closing_gap_max_step_mm':s['closing_step'],
                              'continuous_collision_detection':False,'all_sample_summaries':summaries}}
        if out:
            out=Path(out); out.mkdir(parents=True,exist_ok=True)
            (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            with gzip.open(out/'pairs.jsonl.gz','wt') as f:
                for row in raw: f.write(json.dumps(row,separators=(',',':'))+'\n')
            display = {**obstacles,'ganten_348_bottle':bottle.moved(bt)}
            ts = transformations(wrist,links,joints,metrics,d['open_gap'])
            display.update({name:links[name].moved(t) for name,t in ts.items()})
            if label=='nominal': export(display,out/'meshes')
        return result
    finally:
        engine.close()


def variants(c):
    yield 'nominal',copy.deepcopy(c)
    for delta in [-10.,10.]:
        v=copy.deepcopy(c)
        v['reported']['upper_bottom']+=delta
        v['reported']['rack_height']+=delta
        yield f'height_{delta:+g}',v
    for z in [151.,165.]:
        v=copy.deepcopy(c); v['reported']['rack_height']=z
        yield f'outer_height_{z:g}',v
    for width in [82.,92.,112.]:
        v=copy.deepcopy(c); v['reported']['slot_width']=width
        anchor=c['draft']['upper_centers'][2]
        v['draft']['upper_centers']=[anchor-2*width,anchor-width,anchor]
        v['draft']['lower_centers']=[anchor-2.5*width+i*width for i in range(4)]
        yield f'width_{width:g}',v
    for depth in [14.5,19.5,29.5,34.5]:
        v=copy.deepcopy(c); v['reported']['slot_depth']=depth
        v['reported']['rack_height']=max(c['reported']['rack_height'],c['reported']['upper_bottom']+depth+1)
        yield f'depth_{depth:g}',v
    for key,values in [('bottle_offset_x',[-5.,5.]),('bottle_bottom_y',[140.,180.]),
                       ('bottle_pitch',[-0.1745329252,0.1745329252]),
                       ('bottle_yaw',[-0.1745329252,0.1745329252]),
                       ('tool_roll',[-0.2617993878,0.2617993878])]:
        for value in values:
            v=copy.deepcopy(c);v['draft'][key]=value
            yield f'{key}_{value:+.5f}',v
    v=copy.deepcopy(c);v['draft']['profile']='v'
    yield 'same_width_depth_v_profile',v
    v=copy.deepcopy(c);v['reported']['bottle_diameter']=56.
    yield 'cylinder_at_grip_diameter_56',v


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path)
    parser.add_argument('--description',type=Path)
    parser.add_argument('--backend',type=Path,default=Path(__file__).with_name('model01_proximity'))
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--sweep',action='store_true')
    args=parser.parse_args()
    if args.description is None:
        from ament_index_python.packages import get_package_share_directory
        args.description=Path(get_package_share_directory('d1t_description'))
    c=load_config(args.config or args.description/'config/model01.yaml')
    results=[]; axis=None
    cases=list(variants(c)) if args.sweep else [('nominal',c)]
    for label,params in cases:
        result=scenario(params,args.description,args.backend,args.out/label,label,axis)
        if axis is None: axis=result['axis_x_mm']
        results.append({k:result[k] for k in ['label','axis_x_mm','bottle_bottom_m','grip_reference_m','target','phases']})
        print(label,json.dumps(result['phases']),flush=True)
    (args.out/'sweep.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':
    main()
