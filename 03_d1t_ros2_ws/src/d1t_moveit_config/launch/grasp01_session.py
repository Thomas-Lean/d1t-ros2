#!/usr/bin/env python3
"""Own one domain-7 fake stack, run bounded jobs, stop only its process group."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time
from grasp01_core import guard_environment, active_node_names


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--rviz',action='store_true')
    p.add_argument('--sim-execute',action='store_true',help='explicitly permit gated fake execution; default plan-only')
    p.add_argument('--hold-seconds',type=float,default=0.)
    p.add_argument('--repeat',type=int,choices=[1,2],default=2)
    p.add_argument('--config',action='append',type=Path,help='explicit DRAFT config file; omit to run the bounded four-case set')
    p.add_argument('--installation',type=Path,help='versioned installation input; default frozen VISION snapshot')
    p.add_argument('--load-only',action='store_true',help='load/read back scene only, without planning')
    args=p.parse_args();guard_environment()
    if args.load_only and args.sim_execute:p.error('--load-only forbids execution')
    args.out.mkdir(parents=True,exist_ok=True)
    if 'GRASP01_TOKEN' not in os.environ:
        env=dict(os.environ,GRASP01_TOKEN=secrets.token_hex(24))
        os.execve(sys.executable,[sys.executable,__file__]+sys.argv[1:],env)
    import rclpy
    rclpy.init();probe=rclpy.create_node('grasp01_occupancy_probe')
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(probe,timeout_sec=.1)
        nodes=[n for n in active_node_names(probe) if n!='grasp01_occupancy_probe']
        topics=[n for n,_ in probe.get_topic_names_and_types()]
        evidence={'domain':os.environ['ROS_DOMAIN_ID'],'dds':os.environ['CYCLONEDDS_URI'],'nodes':nodes,'topics':topics}
        (args.out/'occupancy.json').write_text(json.dumps(evidence,indent=2)+'\n')
        if nodes or '/arm_Feedback' in topics:
            print('GRASP01_DOMAIN_BUSY: no stack started; existing nodes untouched');return 4
    finally:
        probe.destroy_node();rclpy.shutdown()
    root=args.root.resolve();ws=root/'03_d1t_ros2_ws';docs=root/'forlearning/GRASP-01-scene-simulation'
    installation=(args.installation or docs/'inputs/installation.confirmed.json').resolve()
    # q_finger comes from actual mesh open gap; bridge gap is based on 66 mm.
    from grasp01_core import model_import
    geom,_=model_import(ws/'src/d1t_description')
    links,joints,tcp=geom.urdf_gripper(ws/'src/d1t_description')
    metrics=geom.gripper_metrics(links,joints,tcp)
    qfinger=(metrics['urdf_open_gap_mm']-60.8)/2000
    raw=((.033-qfinger)*2000-19.8)/.85
    config_paths=args.config or [docs/'config/nominal.json',docs/'config/roll15.json',docs/'config/bottle_forward20.json',docs/'config/rack_yaw180.json']
    config=json.loads(config_paths[0].read_text())
    (args.out/'input-files.json').write_text(json.dumps({str(path.resolve()):hashlib.sha256(path.read_bytes()).hexdigest() for path in [installation,*config_paths]},indent=2)+'\n')
    angles=[v*s for v,s in zip(config['initial_urdf_deg'],[1,1,1,-1,1,-1])]+[raw]
    env=dict(os.environ,GRASP01_INITIAL=','.join(map(str,angles)),GRASP01_RVIZ='1' if args.rviz else '0')
    env['GRASP01_INSTALLATION']=str(installation)
    env['PYTHONPATH']=str(ws/'src/d1t_moveit_config/launch')+':'+env.get('PYTHONPATH','')
    log=(args.out/'stack.log').open('w')
    proc=subprocess.Popen(['ros2','launch','d1t_moveit_config','grasp01_sim.launch.py'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    owner={'pid':os.getpid(),'token':os.environ['GRASP01_TOKEN'],'stack_pid':proc.pid,'nodes':[],'initial_controller_deg':angles}
    codes=[]
    try:
        rclpy.init();node=rclpy.create_node('grasp01_session_probe')
        try:
            deadline=time.monotonic()+30
            required={'fake_d1t','d1t_feedback_node','d1t_arm','d1t_trajectory','move_group','joint_state_bridge','robot_state_publisher','grasp01_desk_visual'}
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.2)
                seen=set(active_node_names(node))-{'grasp01_session_probe'}
                if required<=seen:break
                if proc.poll() is not None:raise RuntimeError('stack exited early')
            if not required<=seen:raise RuntimeError('stack missing '+str(required-seen))
            # MoveIt creates controller-manager and private helper nodes in this process.
            extra=seen-required-{'grasp01_rviz'}
            helper_prefixes=('transform_listener_impl_','move_group_private_','rviz_')
            if args.rviz:helper_prefixes+=('grasp01_rviz_private_','interactive_marker_display_')
            if any(n != 'moveit_simple_controller_manager' and not n.startswith(helper_prefixes) for n in extra):raise RuntimeError('unknown nodes after launch: '+str(extra))
            # Controller-manager node may appear after the startup snapshot.
            owner['nodes']=sorted(seen | {'moveit_simple_controller_manager'} | ({'grasp01_rviz'} if args.rviz else set()))
        finally:
            node.destroy_node();rclpy.shutdown()
        ownerpath=args.out/'owner.json';ownerpath.write_text(json.dumps(owner,indent=2)+'\n')
        (args.out/'initial.json').write_text(json.dumps({'urdf_deg':config['initial_urdf_deg'],'finger_q_m':qfinger,'controller_deg':angles},indent=2)+'\n')
        for repeat in range(1,args.repeat+1):
            for config_path in config_paths:
                label=config_path.stem
                out=args.out/f'run{repeat}-{label}'
                cmd=[sys.executable,str(ws/'src/d1t_moveit_config/launch/grasp01_ros.py'),'--config',str(config_path),'--installation',str(installation),'--description',str(ws/'src/d1t_description'),'--backend',str(ws/'install/d1t_description/lib/d1t_description/model01_proximity'),'--srdf',str(ws/'src/d1t_moveit_config/config/d1t.srdf'),'--out',str(out),'--owner',str(ownerpath)]
                cmd+=['--sim-execute'] if args.sim_execute else ['--plan-only']
                if args.load_only:cmd+=['--load-only']
                with (args.out/f'run{repeat}-{label}.log').open('w') as f:
                    rc=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT).returncode
                codes.append({'repeat':repeat,'label':label,'exit_code':rc});print(json.dumps(codes[-1]),flush=True)
                if rc not in (0,3):raise RuntimeError('integration error; inspect run log')
        if args.hold_seconds:
            print('OWNED_STACK_READY_FOR_RVIZ_CAPTURE',flush=True)
            deadline=time.monotonic()+args.hold_seconds
            while time.monotonic()<deadline and proc.poll() is None:time.sleep(.2)
        return 0
    finally:
        # Signal only the process group we created, while its leader is alive.
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()
        (args.out/'exit-codes.json').write_text(json.dumps({'runs':codes,'stack_returncode':proc.returncode,'cleanup':'only owned stack process group signalled'},indent=2)+'\n')

if __name__=='__main__':raise SystemExit(main())
