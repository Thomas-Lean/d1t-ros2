#!/usr/bin/env python3
"""GRASP-01 isolated scene/plan entry. Default plan-only; fail closed."""
import argparse
from contextlib import contextmanager
import copy
import gzip
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
from grasp01_core import (BOTTLE, FINGERS, SUPPORT, JOINTS, STAGES, SceneGeometry,
                          attachment_pose, densify, digest, guard_environment, active_node_names,
                          guard_graph, require_complete_plan, verify_bottle_count)


def ros_pose(t):
    from geometry_msgs.msg import Pose
    p=Pose()
    p.position.x,p.position.y,p.position.z=map(float,t[:3,3])
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,Rotation.from_matrix(t[:3,:3]).as_quat())
    return p


def pose_matrix(p):
    t=np.eye(4); t[:3,3]=[p.position.x,p.position.y,p.position.z]
    q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    t[:3,:3]=Rotation.from_quat(q).as_matrix() if np.linalg.norm(q)>1e-10 else np.eye(3)
    return t


def mesh_object(name, mesh, matrix, frame='base_link'):
    from geometry_msgs.msg import Point
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle
    obj=CollisionObject(id=name,operation=CollisionObject.ADD)
    obj.header.frame_id=frame
    obj.pose.orientation.w=1.
    obj.meshes=[Mesh(vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in mesh.vertices],triangles=[MeshTriangle(vertex_indices=f.tolist()) for f in mesh.faces])]
    obj.mesh_poses=[ros_pose(matrix)]
    return obj


def box_object(spec):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    obj=CollisionObject(id=spec['id'],operation=CollisionObject.ADD)
    obj.header.frame_id=spec['frame_id']; obj.pose.orientation.w=1.
    obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(map(float,spec['size_m'])))]
    matrix=np.asarray(spec['matrix']) if 'matrix' in spec else np.eye(4)
    if 'center_m' in spec:matrix[:3,3]=spec['center_m']
    obj.primitive_poses=[ros_pose(matrix)]
    return obj


def scene_diff(geometry, existing_ids):
    from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, ObjectColor
    from std_msgs.msg import ColorRGBA
    scene=PlanningScene(is_diff=True,name='GRASP-01 DRAFT_SIMULATION')
    scene.robot_state.is_diff=True
    scene.world.collision_objects=[mesh_object(n,geometry.meshes[n],t) for n,t in geometry.world.items() if n!='desk']
    scene.world.collision_objects.append(box_object(geometry.desk_spec))
    for name in ('mount_board','desk_clamp'):
        if name in existing_ids:scene.world.collision_objects.append(CollisionObject(id=name,operation=CollisionObject.REMOVE))
    scene.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name='Empty_Link6',object=box_object(geometry.camera_spec),touch_links=['Empty_Link6'])]
    for name in geometry.world:
        color=(.12,.45,.9,1.) if name==BOTTLE else ((.55,.55,.55,.6) if name=='desk' else (.2,.72,.65,1.))
        # Show the physical desktop via a separate Marker; do not draw the
        # padded collision volume as if it were a 30-mm-higher solid table.
        if name=='desk' and hasattr(geometry,'physical_desk_spec'):color=(.55,.55,.55,0.)
        scene.object_colors.append(ObjectColor(id=name,color=ColorRGBA(r=color[0],g=color[1],b=color[2],a=color[3])))
    scene.object_colors.append(ObjectColor(id='vision_assembly',color=ColorRGBA(r=.9,g=.4,b=.1,a=.9)))
    return scene


def attachment_diff(geometry, world_link, world_bottle, attach=True):
    from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
    scene=PlanningScene(is_diff=True);scene.robot_state.is_diff=True
    if attach:
        # MoveIt processes the attached body first and removes its world copy.
        # A second world REMOVE in this diff fails after attachment succeeded.
        obj=mesh_object(BOTTLE,geometry.bottle,attachment_pose(world_link,world_bottle),'Empty_Link6')
        scene.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name='Empty_Link6',object=obj,touch_links=FINGERS)]
    else:
        scene.robot_state.attached_collision_objects=[AttachedCollisionObject(link_name='Empty_Link6',object=CollisionObject(id=BOTTLE,operation=CollisionObject.REMOVE))]
        scene.world.collision_objects=[mesh_object(BOTTLE,geometry.bottle,world_bottle)]
    return scene


def scene_signature(scene):
    from rosidl_runtime_py.convert import message_to_ordereddict
    # Strip moving joint feedback/stamps, retain collision geometry, attachment and ACM.
    data=message_to_ordereddict(scene)
    keep={k:data[k] for k in ('world','allowed_collision_matrix','link_padding','link_scale')}
    keep['attached']=data['robot_state']['attached_collision_objects']
    def strip(v):
        if isinstance(v,dict):return {k:strip(x) for k,x in v.items() if k!='stamp'}
        if isinstance(v,list):return [strip(x) for x in v]
        # MoveIt attachment transform round-trips introduce floating-point noise.
        # Canonicalize to 1e-9 (1 nm for metre coordinates), including signed zero.
        if isinstance(v,float):
            rounded=round(v,9)
            return 0.0 if rounded==0 else rounded
        return v
    return digest(strip(keep))


def fixed_mount_matrix(original):
    """Add only the operator-authorized fixed base_link/desk contact pair."""
    from moveit_msgs.msg import AllowedCollisionEntry
    matrix=copy.deepcopy(original)
    for name in ('base_link','desk'):
        if name not in matrix.entry_names:
            matrix.entry_names.append(name)
            for row in matrix.entry_values:row.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
    i=matrix.entry_names.index('base_link');j=matrix.entry_names.index('desk')
    matrix.entry_values[i].enabled[j]=True
    matrix.entry_values[j].enabled[i]=True
    return matrix


class Client:
    def __init__(self, node, geometry, output):
        from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity, GetPositionIK, GetCartesianPath
        self.node=node;self.g=geometry;self.out=Path(output);self.out.mkdir(parents=True,exist_ok=True)
        self.clients={name:node.create_client(kind,name) for name,kind in [('/apply_planning_scene',ApplyPlanningScene),('/get_planning_scene',GetPlanningScene),('/check_state_validity',GetStateValidity),('/compute_ik',GetPositionIK),('/compute_cartesian_path',GetCartesianPath)]}
        self.actions={};self.motion_requests=0;self.expected_scene=None;self.owner=None

    def call(self,name,request,timeout=20.):
        import rclpy
        c=self.clients[name]
        if not c.wait_for_service(timeout_sec=timeout):raise RuntimeError(name+' unavailable')
        f=c.call_async(request);rclpy.spin_until_future_complete(self.node,f,timeout_sec=timeout)
        if not f.done() or f.result() is None:raise RuntimeError(name+' timeout; no retry')
        return f.result()

    def save(self,name,msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        # Large mesh scenes otherwise spend significant time writing JSON token by
        # token and compressing at level 9 on the synchronous motion-control path.
        payload=json.dumps(message_to_ordereddict(msg),separators=(',',':')).encode()
        with gzip.open(self.out/(name+'.json.gz'),'wb',compresslevel=1) as f:f.write(payload)

    def read(self):
        from moveit_msgs.srv import GetPlanningScene
        r=GetPlanningScene.Request();r.components.components=1023
        return self.call('/get_planning_scene',r).scene

    def apply(self,scene):
        from moveit_msgs.srv import ApplyPlanningScene
        if not self.call('/apply_planning_scene',ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('scene application failed; state requires readback, no auto-reset')

    def load(self):
        before=self.read(); self.save('scene-before',before)
        old={o.id for o in before.world.collision_objects}
        if BOTTLE in [a.object.id for a in before.robot_state.attached_collision_objects]:
            raise RuntimeError('bottle already attached: explicit owned reset required')
        diff=scene_diff(self.g,old)
        if self.g.config.get('allow_fixed_base_desk_contact') is True:
            diff.allowed_collision_matrix=fixed_mount_matrix(before.allowed_collision_matrix)
        self.apply(diff);after=self.read();self.save('scene-after',after)
        if self.g.config.get('allow_fixed_base_desk_contact') is True and after.allowed_collision_matrix!=diff.allowed_collision_matrix:
            raise RuntimeError('fixed mounting contact matrix readback mismatch')
        world={o.id:o for o in after.world.collision_objects}; attached={o.object.id:o for o in after.robot_state.attached_collision_objects}
        verify_bottle_count(list(world),list(attached),False)
        if set(self.g.world)-set(world) or {'mount_board','desk_clamp'}&set(world):raise RuntimeError('scene IDs mismatch')
        if (old-set(self.g.world)-{'mount_board','desk_clamp'})-set(world):raise RuntimeError('unrelated object removed')
        for name,t in self.g.world.items():
            o=world[name]
            actual=pose_matrix(o.pose)@pose_matrix((o.primitive_poses if name=='desk' else o.mesh_poses)[0])
            if not np.allclose(actual,t,atol=1e-8,rtol=0):raise RuntimeError('double transform/readback pose mismatch: '+name)
            if name=='desk':
                if not np.allclose(o.primitives[0].dimensions,self.g.desk_spec['size_m']):raise RuntimeError('desk size mismatch')
            else:
                verts=np.array([[v.x,v.y,v.z] for v in o.meshes[0].vertices])
                expected=self.g.meshes[name].vertices
                # MoveIt may deduplicate/reorder vertices; compare sorted geometry.
                if verts.shape!=expected.shape or not np.allclose(verts[np.lexsort(verts.T)],expected[np.lexsort(expected.T)],atol=1e-8):raise RuntimeError('mesh readback mismatch: '+name)
        camera=attached['vision_assembly']
        if camera.touch_links!=['Empty_Link6'] or camera.link_name!='Empty_Link6':raise RuntimeError('camera touch links changed')
        cam_t=pose_matrix(camera.object.pose)@pose_matrix(camera.object.primitive_poses[0])
        if not np.allclose(cam_t,self.g.camera_local,atol=1e-9):raise RuntimeError('camera pose mismatch')
        self.apply(diff);again=self.read();self.save('scene-repeat',again)
        if scene_signature(after)!=scene_signature(again):raise RuntimeError('repeated load not idempotent')
        return after

    @contextmanager
    def contact_pairs(self, partners, *, pairs=None):
        from moveit_msgs.msg import PlanningScene, AllowedCollisionEntry
        original=copy.deepcopy(self.read().allowed_collision_matrix)
        matrix=copy.deepcopy(original)
        pairs=[(BOTTLE,partner) for partner in partners] if pairs is None else pairs
        for name in [name for pair in pairs for name in pair]:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        for a,b in pairs:
            i=matrix.entry_names.index(a);j=matrix.entry_names.index(b)
            matrix.entry_values[i].enabled[j]=True;matrix.entry_values[j].enabled[i]=True
        diff=PlanningScene(is_diff=True,allowed_collision_matrix=matrix);diff.robot_state.is_diff=True
        try:
            self.apply(diff)
            yield
        finally:
            restore=PlanningScene(is_diff=True,allowed_collision_matrix=original);restore.robot_state.is_diff=True
            self.apply(restore)
            if self.read().allowed_collision_matrix!=original:
                raise RuntimeError('contact matrix restoration failed; no further stages')

    def finger_contact(self):
        return self.contact_pairs(FINGERS)

    def finger_rack_contact(self, phase):
        from contextlib import nullcontext
        if self.g.config.get('allow_finger_upper_plate_contact') is True and phase in ('approach','close','lift'):
            return self.contact_pairs([],pairs=[(finger,'ganten_rack_upper_plate') for finger in FINGERS])
        return nullcontext()

    def state(self,q,attached=None):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState
        s=RobotState(is_diff=True,joint_state=JointState(name=JOINTS+['Joint_L','Joint_R'],position=list(map(float,q))))
        if attached is not None:s.attached_collision_objects=[attached]
        return s

    def validity(self,q):
        from moveit_msgs.srv import GetStateValidity
        # Empty group checks whole robot, including fixed base and camera.
        req=GetStateValidity.Request(robot_state=self.state(q),group_name='')
        return self.call('/check_state_validity',req)

    def ik(self,target,q,avoid):
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import PoseStamped
        attempts=[]
        seeds=[q,np.r_[[-math.atan2(target[1,3],target[0,3]),.8,.5,0.,-1.2,0.],q[6:]],np.r_[[-math.atan2(target[1,3],target[0,3]),.3,.9,0.,-1.2,0.],q[6:]]]
        for seed in seeds:
            req=GetPositionIK.Request();r=req.ik_request
            r.group_name='arm';r.ik_link_name='tcp_link';r.avoid_collisions=avoid;r.robot_state=self.state(seed)
            r.pose_stamped=PoseStamped(pose=ros_pose(target));r.pose_stamped.header.frame_id='base_link'
            r.timeout.nanosec=300000000
            res=self.call('/compute_ik',req);attempts.append(res.error_code.val)
            if res.error_code.val==1:
                vals=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
                return np.array([vals[n] for n in JOINTS]+list(q[6:])),attempts
        return None,attempts

    def action(self,name,kind,goal,timeout=30.):
        import rclpy
        from rclpy.action import ActionClient
        if name not in self.actions:self.actions[name]=ActionClient(self.node,kind,name)
        ac=self.actions[name]
        if not ac.wait_for_server(timeout_sec=10.):raise RuntimeError(name+' unavailable')
        f=ac.send_goal_async(goal);rclpy.spin_until_future_complete(self.node,f,timeout_sec=10.)
        h=f.result()
        if h is None or not h.accepted:raise RuntimeError('action goal rejected')
        try:
            f=h.get_result_async();rclpy.spin_until_future_complete(self.node,f,timeout_sec=timeout)
            if not f.done():raise RuntimeError('action timeout')
            return f.result().result
        except BaseException:
            cf=h.cancel_goal_async();rclpy.spin_until_future_complete(self.node,cf,timeout_sec=3.)
            raise  # cancellation is not an immediate controller stop

    def plan_joint(self,q,target):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints,JointConstraint
        goal=MoveGroup.Goal();r=goal.request;r.group_name='arm';r.start_state=self.state(q)
        r.num_planning_attempts=1;r.allowed_planning_time=5.
        r.max_velocity_scaling_factor=.15;r.max_acceleration_scaling_factor=.15
        r.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=float(v),tolerance_above=.001,tolerance_below=.001,weight=1.) for n,v in zip(JOINTS,target)])]
        goal.planning_options.plan_only=True
        return self.action('/move_action',MoveGroup,goal)

    def cartesian(self,q,target,*,avoid_collisions=True):
        from moveit_msgs.srv import GetCartesianPath
        req=GetCartesianPath.Request();req.header.frame_id='base_link';req.start_state=self.state(q)
        req.group_name='arm';req.link_name='tcp_link';req.max_step=.001;req.jump_threshold=5.;req.avoid_collisions=bool(avoid_collisions)
        req.waypoints=[ros_pose(target)]
        return self.call('/compute_cartesian_path',req)

    def audit_trajectory(self,trajectory,q,label,attached=None):
        jt=trajectory.joint_trajectory
        indices=[list(jt.joint_names).index(n) for n in JOINTS]
        rows=[list(q)]+[[p.positions[i] for i in indices]+list(q[6:]) for p in jt.points]
        dense=densify(rows)
        records=[]
        print(f'AUDIT {label}: {len(dense)} samples',flush=True)
        for sample in dense:
            offline=self.g.check(sample,attached=attached,phase=label)
            valid=self.validity(sample)
            records.append({'q':sample.tolist(),'moveit_valid':valid.valid,'offline':offline})
            if len(records)%100==0:print(f'AUDIT {label}: {len(records)}/{len(dense)}',flush=True)
            if offline['blocked'] or not valid.valid:break
        with gzip.open(self.out/(label+'-sweep.json.gz'),'wt') as f:json.dump(records,f)
        return bool(records) and len(records)==len(dense) and not records[-1]['offline']['blocked'] and records[-1]['moveit_valid']

    def end_q(self,trajectory,q):
        jt=trajectory.joint_trajectory
        if not jt.points:raise RuntimeError('empty trajectory')
        return np.array([jt.points[-1].positions[list(jt.joint_names).index(n)] for n in JOINTS]+list(q[6:]))

    def close_geometry(self,q):
        rows=[]
        with self.finger_contact(), self.finger_rack_contact('close'):
            for finger in np.linspace(q[6],self.g.closed_q,max(1,math.ceil(abs(self.g.closed_q-q[6])/.000125))+1):
                sample=np.r_[q[:6],finger,finger]
                check=self.g.check(sample,phase='close')
                valid=self.validity(sample)
                rows.append({'q':sample.tolist(),'geometry':check,'moveit_valid':valid.valid})
                if check['blocked'] or not valid.valid:break
        with gzip.open(self.out/'close-sweep.json.gz','wt') as f:json.dump(rows,f)
        if rows[-1]['geometry']['blocked'] or not rows[-1]['moveit_valid']:raise RuntimeError('closure blocked')
        return np.array(rows[-1]['q'])

    def attach_at(self,q):
        # Planning-only hypothetical state: continuity evaluated at planned q.
        link=self.g.robot.fk(q)['Empty_Link6'];world=self.g.world[BOTTLE]
        relative=attachment_pose(link,world)
        self.apply(attachment_diff(self.g,link,world))
        observed=self.read();self.save('attached-planning-state',observed)
        verify_bottle_count([o.id for o in observed.world.collision_objects],[o.object.id for o in observed.robot_state.attached_collision_objects],True)
        item=next(a for a in observed.robot_state.attached_collision_objects if a.object.id==BOTTLE)
        actual=pose_matrix(item.object.pose)@pose_matrix(item.object.mesh_poses[0])
        if item.touch_links!=FINGERS or not np.allclose(link@actual,world,atol=1e-8):raise RuntimeError('attachment continuity mismatch')
        return relative

    def current_q(self):
        import rclpy
        from sensor_msgs.msg import JointState
        values=[]
        sub=self.node.create_subscription(JointState,'/joint_states',lambda msg: values.append(msg),10)
        try:
            end=time.monotonic()+2.
            while not values and time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.05)
            if not values:raise RuntimeError('fresh joint feedback timeout')
            vals=dict(zip(values[-1].name,values[-1].position))
            return np.array([vals[n] for n in JOINTS+['Joint_L','Joint_R']])
        finally:self.node.destroy_subscription(sub)

    def execute_trajectory(self,trajectory,q,label,attached=None):
        from moveit_msgs.action import ExecuteTrajectory
        from sensor_msgs.msg import JointState
        guard_environment()
        guard_graph(active_node_names(self.node),sorted(i.node_name for i in self.node.get_publishers_info_by_topic('/arm_Feedback')),sorted(i.node_name for i in self.node.get_subscriptions_info_by_topic('/arm_Command')),self.owner['nodes']+['grasp01_client'])
        if scene_signature(self.read())!=self.expected_scene:raise RuntimeError('scene changed before execution')
        if not np.allclose(q,self.current_q(),atol=math.radians(.2),rtol=0):raise RuntimeError('execution start mismatch')
        if len(trajectory.joint_trajectory.points)>400:raise RuntimeError('trajectory exceeds point-controller capacity')
        samples=[]
        def record(msg):
            vals=dict(zip(msg.name,msg.position))
            if all(n in vals for n in JOINTS+['Joint_L','Joint_R']):samples.append([vals[n] for n in JOINTS+['Joint_L','Joint_R']])
        sub=self.node.create_subscription(JointState,'/joint_states',record,100)
        try:
            self.motion_requests+=1
            res=self.action('/execute_trajectory',ExecuteTrajectory,ExecuteTrajectory.Goal(trajectory=trajectory),timeout=300.)
        finally:self.node.destroy_subscription(sub)
        self.save(label+'-execute-result',res)
        if res.error_code.val!=1 or not samples:raise RuntimeError('execution failed or feedback missing')
        # Post-execution swept check is evidence, not a real-time stop guarantee.
        audit=[{'q':x.tolist(),'geometry':self.g.check(x,attached=attached,phase=label)} for x in densify(samples)]
        with gzip.open(self.out/(label+'-fake-feedback.json.gz'),'wt') as f:json.dump(audit,f)
        if any(x['geometry']['blocked'] for x in audit):raise RuntimeError('fake feedback sweep collided; no dependent stage')
        expected=self.end_q(trajectory,q)
        if np.max(np.abs(self.current_q()[:6]-expected[:6]))>math.radians(.2):raise RuntimeError('fake arrival mismatch')
        return expected


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('config','installation','description','backend','srdf','out','owner'):
        p.add_argument('--'+k,required=True,type=Path)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--sim-execute',action='store_true');mode.add_argument('--plan-only',action='store_true')
    mode.add_argument('--approach-only',action='store_true',help='validate and preview pregrasp/approach only; no closure or execution')
    p.add_argument('--load-only',action='store_true')
    a=p.parse_args();guard_environment()
    owner=json.loads(a.owner.read_text())
    # Only a live launcher carrying our unpredictable run token can own this graph.
    env=Path(f"/proc/{owner['pid']}/environ").read_bytes()
    if ('GRASP01_TOKEN='+owner['token']).encode()+b'\0' not in env:raise ValueError('owner process not authenticated')
    import rclpy
    rclpy.init();node=rclpy.create_node('grasp01_client')
    geom=None
    result={'status':'FAILED_NO_MOTION','motion_requests_sent':0,'stages':[]}
    try:
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        guard_graph(active_node_names(node),sorted(i.node_name for i in node.get_publishers_info_by_topic('/arm_Feedback')),sorted(i.node_name for i in node.get_subscriptions_info_by_topic('/arm_Command')),owner['nodes']+['grasp01_client'])
        geom=SceneGeometry(json.loads(a.config.read_text()),json.loads(a.installation.read_text()),a.description,a.backend,a.srdf)
        client=Client(node,geom,a.out);client.owner=owner
        scene=client.load();result['scene_signature']=scene_signature(scene);result['scene']=geom.metadata()
        if a.load_only:
            result['status']='SCENE_APPLIED_READBACK';return 0
        q=np.r_[np.deg2rad(geom.config['initial_urdf_deg']),geom.open_q,geom.open_q]
        observed_q=client.current_q()
        result['observed_initial_urdf_rad']=observed_q.tolist()
        if np.max(np.abs(observed_q[:6]-q[:6]))>math.radians(.2) or np.max(np.abs(observed_q[6:]-q[6:]))>.0002:
            raise RuntimeError('configured initial state does not match fresh fake feedback')
        initial=client.validity(q);client.save('initial-validity',initial)
        off=geom.check(q)
        result['initial']={'moveit_valid':initial.valid,'offline':off}
        result['target_diagnostics']={}
        # Bounded non-motion endpoint diagnostics remain useful after initial failure.
        for name,target in geom.targets().items():
            if a.approach_only and name not in ('pregrasp','approach'):continue
            with client.finger_rack_contact(name):
                sol,attempts=client.ik(target,q,True)
                raw,raw_attempts=(None,[]) if sol is not None else client.ik(target,q,False)
                diagnostic=sol if sol is not None else raw
                row={'avoid_codes':attempts,'diagnostic_no_avoid_codes':raw_attempts,'q':None if diagnostic is None else diagnostic.tolist(),'planning_permission':sol is not None}
                if diagnostic is not None:
                    row['geometry']=geom.check(diagnostic,phase=name)
                    state=client.validity(diagnostic);client.save(name+'-validity',state);row['moveit_valid']=state.valid
            result['target_diagnostics'][name]=row
        if off['blocked'] or not initial.valid:
            result['status']='BLOCKED_INITIAL_STATE'
            result['stages']=[{'stage':s,'ok':False,'reason':'initial full-arm state invalid; dependent stage not run'} for s in STAGES]
            return 3
        pre=result['target_diagnostics']['pregrasp']['q']
        if pre is None:raise RuntimeError('pregrasp IK has no solution in 3 seeds')
        plan=client.plan_joint(q,pre);client.save('pregrasp-plan',plan)
        ok=plan.error_code.val==1 and client.audit_trajectory(plan.planned_trajectory,q,'pregrasp')
        result['stages'].append({'stage':'pregrasp','ok':ok,'error_code':plan.error_code.val})
        if not ok:raise RuntimeError('pregrasp path invalid')
        with client.finger_rack_contact('approach'):
            cart=client.cartesian(pre,geom.targets()['approach']);client.save('approach-plan',cart)
            ok=cart.error_code.val==1 and cart.fraction>=1.-1e-9 and client.audit_trajectory(cart.solution,pre,'approach')
        result['stages'].append({'stage':'approach','ok':ok,'fraction':cart.fraction})
        if not ok:raise RuntimeError('approach partial or colliding')
        for point in cart.solution.joint_trajectory.points:
            duration=(point.time_from_start.sec+point.time_from_start.nanosec*1e-9)/.15
            point.time_from_start.sec=int(duration);point.time_from_start.nanosec=int((duration%1)*1e9)
            point.velocities=[v*.15 for v in point.velocities];point.accelerations=[v*.15*.15 for v in point.accelerations]
        if a.approach_only:
            from moveit_msgs.msg import DisplayTrajectory
            pub=node.create_publisher(DisplayTrajectory,'/display_planned_path',1)
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            result['preview_subscribers']=pub.get_subscription_count()
            pub.publish(DisplayTrajectory(model_id='d1_550_description',trajectory_start=client.state(q),
                                          trajectory=[plan.planned_trajectory,cart.solution]))
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            result['status']='PREGRASP_APPROACH_PLAN_ONLY'
            return 0
        q_grasp=client.end_q(cart.solution,pre)
        closed=client.close_geometry(q_grasp)
        result['stages'].append({'stage':'close','ok':True,'kind':'geometric closure; no grip service called'})
        attached=client.attach_at(closed)
        result['stages'].append({'stage':'attach','ok':True,'kind':'hypothetical planning state only'})
        trajectories={'pregrasp':plan.planned_trajectory,'approach':cart.solution}
        starts={'pregrasp':q,'approach':np.array(pre)}
        state=closed
        for stage in ['lift','exit','upright']:
            starts[stage]=state.copy()
            from contextlib import nullcontext
            joint_upright=stage=='upright' and geom.config.get('upright_planner')=='joint'
            with (client.contact_pairs(SUPPORT) if stage=='lift' else nullcontext()), client.finger_rack_contact(stage):
                if joint_upright:
                    target,attempts=client.ik(geom.targets()[stage],state,True)
                    if target is None:raise RuntimeError('upright joint-planning target IK failed: '+str(attempts))
                    segment=client.plan_joint(state,target);client.save(stage+'-plan',segment)
                    solution=segment.planned_trajectory
                    ok=segment.error_code.val==1 and client.audit_trajectory(solution,state,stage,attached)
                    stage_result={'stage':stage,'ok':ok,'planner':'joint','error_code':segment.error_code.val}
                else:
                    segment=client.cartesian(state,geom.targets()[stage]);client.save(stage+'-plan',segment)
                    solution=segment.solution
                    ok=segment.error_code.val==1 and segment.fraction>=1.-1e-9 and client.audit_trajectory(solution,state,stage,attached)
                    stage_result={'stage':stage,'ok':ok,'planner':'cartesian','fraction':segment.fraction}
            result['stages'].append(stage_result)
            if not ok:raise RuntimeError(stage+' path incomplete/colliding; hypothetical attached state retained for diagnosis')
            # Apply the same time-scaling relation used by s4b4_tcp, without thinning.
            for point in ([] if joint_upright else solution.joint_trajectory.points):
                duration=(point.time_from_start.sec+point.time_from_start.nanosec*1e-9)/.15
                point.time_from_start.sec=int(duration);point.time_from_start.nanosec=int((duration%1)*1e9)
                point.velocities=[v*.15 for v in point.velocities];point.accelerations=[v*.15*.15 for v in point.accelerations]
            trajectories[stage]=solution
            state=client.end_q(solution,state)
        # Successful planning returns the planning scene to its initial state.
        # Failed planning never silently resets; the finally block records readback.
        client.apply(attachment_diff(geom,geom.robot.fk(closed)['Empty_Link6'],geom.world[BOTTLE],False))
        observed=client.read()
        require_complete_plan(result['stages'],result['scene_signature'],scene_signature(observed),q,client.current_q())
        client.expected_scene=scene_signature(observed)
        result['status']='FULL_CHAIN_PLAN_ONLY'
        if a.sim_execute:
            from d1t_interfaces.srv import Grip
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
            # All motion clients are created only after the complete-plan gate.
            client.execute_trajectory(trajectories['pregrasp'],starts['pregrasp'],'pregrasp')
            with client.finger_rack_contact('approach'):
                client.expected_scene=scene_signature(client.read())
                client.execute_trajectory(trajectories['approach'],starts['approach'],'approach')
            client.expected_scene=scene_signature(client.read())
            raw=((.033-geom.closed_q)*2000-19.8)/.85
            client.clients['/fake_d1t/set_parameters']=node.create_client(SetParameters,'/fake_d1t/set_parameters')
            req=SetParameters.Request(parameters=[Parameter(name='grip_block_raw',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(raw)))])
            response=client.call('/fake_d1t/set_parameters',req)
            if not response.results or not all(r.successful for r in response.results):raise RuntimeError('fake contact injection failed')
            try:
                guard_environment()
                guard_graph(active_node_names(node),sorted(i.node_name for i in node.get_publishers_info_by_topic('/arm_Feedback')),sorted(i.node_name for i in node.get_subscriptions_info_by_topic('/arm_Command')),owner['nodes']+['grasp01_client'])
                if scene_signature(client.read())!=client.expected_scene:raise RuntimeError('scene changed before grip')
                with client.finger_rack_contact('close'),client.finger_contact():
                    client.clients['/grip']=node.create_client(Grip,'/grip')
                    client.motion_requests+=1
                    grip=client.call('/grip',Grip.Request(target_deg=float(raw-5),speed_ratio=.15,timeout_sec=10.),timeout=15.)
                    client.save('grip-result',grip)
                    if not grip.success or grip.reason!='ok' or grip.state!='grasped':raise RuntimeError('fake grip did not classify grasped; no attachment')
            except BaseException:
                result['fake_injection_retained']='grip_block_raw retained until owned stack cleanup; removing it would continue closing toward pending target'
                raise
            actual=client.current_q()
            if not np.allclose(actual,closed,atol=.0002,rtol=0):raise RuntimeError('closed fake joints differ from geometry')
            client.attach_at(actual)
            client.expected_scene=scene_signature(client.read())
            for stage in ['lift','exit','upright']:
                with (client.contact_pairs(SUPPORT) if stage=='lift' else nullcontext()), client.finger_rack_contact(stage):
                    client.expected_scene=scene_signature(client.read())
                    client.execute_trajectory(trajectories[stage],starts[stage],stage,attached)
            result['fake_injection_retained']='grip_block_raw retained during assumed grasp until owned stack cleanup'
            result['status']='FULL_CHAIN_FAKE_COMPLETED_ASSUMED_GRASP'
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        result['status']='FAILED_NO_DEPENDENT_MOTION' if 'client' in locals() and client.motion_requests else 'FAILED_NO_MOTION'
        result['error']=f'{type(exc).__name__}: {exc}'
        return 2
    finally:
        if 'client' in locals():
            result['motion_requests_sent']=client.motion_requests
            try:client.save('scene-final',client.read())
            except Exception as exc:result['final_readback_error']=str(exc)
        a.out.mkdir(parents=True,exist_ok=True)
        (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({'status':result['status'],'motion_requests_sent':result['motion_requests_sent'],'output':str(a.out)}),flush=True)
        if geom:geom.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
