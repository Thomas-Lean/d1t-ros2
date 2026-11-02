"""GRASP-01 pure contracts and MODEL/VISION adapters. No ROS side effects."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from grasp01_vendor.vision02 import (validate_installation, desk_collision_spec,
                                    camera_mount_collision_spec)

JOINTS = [f'Joint{i}' for i in range(1, 7)]
FINGERS = ['Empty_Link_L', 'Empty_Link_R']
SUPPORT = ['ganten_rack_upper_plate','ganten_rack_upper_front_edge','ganten_rack_upper_back_edge']
BOTTLE = 'ganten_348_bottle'
STAGES = ['pregrasp', 'approach', 'close', 'attach', 'lift', 'exit', 'upright']


def model_import(description):
    # Installed scripts are beside the backend; source checkout supports unit tests.
    p = Path(description)
    scripts = p / 'scripts'
    if not scripts.exists():
        scripts = p.parents[1] / 'lib/d1t_description'
    sys.path.insert(0, str(scripts))
    import model01_geometry as geom
    import model01_analysis as analysis
    return geom, analysis


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def guard_environment(env=None):
    env = os.environ if env is None else env
    if env.get('ROS_DOMAIN_ID') != '7' or env.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        raise ValueError('requires domain 7 and rmw_cyclonedds_cpp before creating ROS clients')
    uri = env.get('CYCLONEDDS_URI', '')
    if not uri.startswith('file://'):
        raise ValueError('DDS configuration must be an existing local file')
    root = ET.parse(uri[7:]).getroot()
    values = lambda tag: [e for e in root.iter() if e.tag.split('}')[-1] == tag]
    interfaces, peers, multicast = values('NetworkInterface'), values('Peer'), values('AllowMulticast')
    if (len(interfaces) != 1 or interfaces[0].get('address') != '127.0.0.1'
            or not peers or any(p.get('address') not in ('localhost', '127.0.0.1') for p in peers)
            or len(multicast) != 1 or multicast[0].text.strip().lower() != 'false'):
        raise ValueError('DDS is not exclusively loopback unicast')
    return hashlib.sha256(Path(uri[7:]).read_bytes()).hexdigest()


def active_node_names(node):
    """Ignore only domain-7 CLI daemons with no application topic endpoints."""
    names=node.get_node_names()
    result=[]
    for name in names:
        if name.startswith('_ros2cli_daemon_7_') and names.count(name)==1:
            endpoints=node.get_publisher_names_and_types_by_node(name,'/')+node.get_subscriber_names_and_types_by_node(name,'/')
            if all(topic in ('/rosout','/parameter_events') for topic,_ in endpoints):
                continue
        result.append(name)
    return result


def guard_graph(nodes, feedback_publishers, command_subscribers, owned_nodes):
    unknown = set(nodes) - set(owned_nodes)
    # RViz's MotionPlanning display creates helper nodes after the launcher has
    # sampled the graph.  Accept only its generated names when our named RViz
    # node is itself owned; ordinary plan-only sessions retain exact ownership.
    if 'grasp01_rviz' in owned_nodes:
        rviz_helpers = ('grasp01_rviz_private_', 'interactive_marker_display_',
                        'transform_listener_impl_')
        unknown = {name for name in unknown if not name.startswith(rviz_helpers)}
    if len(nodes)!=len(set(nodes)):
        raise ValueError('duplicate node identities; graph ownership ambiguous')
    if unknown or feedback_publishers != ['fake_d1t'] or command_subscribers != ['fake_d1t']:
        raise ValueError(f'non-exclusive fake graph: unknown={sorted(unknown)}, feedback={feedback_publishers}, command={command_subscribers}')


def rigid_mm_deg(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', rpy, degrees=True).as_matrix()
    t[:3, 3] = np.asarray(xyz, float) / 1000
    return t


def attachment_pose(world_link, world_object):
    return np.linalg.inv(world_link) @ world_object


def verify_bottle_count(world_ids, attached_ids, attached):
    if list(world_ids).count(BOTTLE) != (0 if attached else 1) or list(attached_ids).count(BOTTLE) != (1 if attached else 0):
        raise ValueError('bottle world/attached uniqueness violated')


def require_complete_plan(stages, expected_scene, current_scene, expected_q, actual_q):
    if expected_scene != current_scene:
        raise ValueError('scene changed since planning')
    if not np.allclose(expected_q, actual_q, atol=math.radians(.2), rtol=0):
        raise ValueError('start state mismatch')
    if [s['stage'] for s in stages] != STAGES or not all(s.get('ok') for s in stages):
        raise ValueError('full chain not validated; refusing all dependent execution')
    if any(s.get('fraction', 1.) < 1. - 1e-9 for s in stages):
        raise ValueError('partial Cartesian path')


def densify(rows, angular_step=math.radians(.25), finger_step=.000125):
    if not len(rows):
        raise ValueError('empty trajectory')
    result = [np.asarray(rows[0], float)]
    for end in rows[1:]:
        end = np.asarray(end, float)
        delta = end - result[-1]
        n = max(1, math.ceil(np.max(np.abs(delta[:6])) / angular_step))
        if len(delta) > 6:
            n = max(n, math.ceil(np.max(np.abs(delta[6:])) / finger_step))
        result.extend(np.linspace(result[-1], end, n + 1)[1:])
    return result


class RobotGeometry:
    """Read-only URDF FK/meshes for independent full-body FCL checks."""
    def __init__(self, description, srdf):
        self.geom, _ = model_import(description)
        root = ET.parse(Path(description) / 'urdf/d1t.urdf').getroot()
        self.meshes, self.joints, self.limits = {}, [], {}
        def origin(e):
            if e is None:
                return np.eye(4)
            t = np.eye(4)
            t[:3, :3] = Rotation.from_euler('xyz', list(map(float, e.get('rpy', '0 0 0').split()))).as_matrix()
            t[:3, 3] = list(map(float, e.get('xyz', '0 0 0').split()))
            return t
        for link in root.findall('link'):
            col = link.find('collision')
            if col is None:
                continue
            mesh = col.find('geometry/mesh')
            m = self.geom.read_stl(Path(description) / mesh.get('filename').split('package://d1t_description/')[1])
            m.vertices *= np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
            self.meshes[link.get('name')] = m.moved(origin(col.find('origin')))
        for j in root.findall('joint'):
            axis = j.find('axis')
            self.joints.append((j.get('name'), j.get('type'), j.find('parent').get('link'), j.find('child').get('link'), origin(j.find('origin')), np.array(list(map(float, axis.get('xyz').split()))) if axis is not None else np.zeros(3)))
            if j.find('limit') is not None:
                self.limits[j.get('name')] = [float(j.find('limit').get(k)) for k in ('lower', 'upper')]
        self.disabled = {frozenset((e.get('link1'), e.get('link2'))) for e in ET.parse(srdf).getroot().findall('disable_collisions')}

    def fk(self, q):
        vals = dict(zip(JOINTS + ['Joint_L', 'Joint_R'], q))
        ts = {'base_link': np.eye(4)}
        for name, kind, parent, child, origin, axis in self.joints:
            d = np.eye(4)
            if kind == 'revolute':
                d[:3, :3] = Rotation.from_rotvec(axis * vals[name]).as_matrix()
            elif kind == 'prismatic':
                d[:3, 3] = axis * vals[name]
            ts[child] = ts[parent] @ origin @ d
        return ts

    def bounds(self, q):
        return [n for n, v in zip(JOINTS + ['Joint_L', 'Joint_R'], q) if not self.limits[n][0] <= v <= self.limits[n][1]]


class SceneGeometry:
    def __init__(self, config, installation, description, backend, srdf):
        self.config = copy.deepcopy(config)
        self.installation = validate_installation(installation)
        self.g, self.a = model_import(description)
        self.robot = RobotGeometry(description, srdf)
        self.model = self.g.load_config(Path(description) / 'config/model01.yaml')
        for key, value in config.get('model_draft', {}).items():
            self.model['draft'][key] = value
        self.g.validate(self.model)
        self.rack, self.post_notes = self.g.rack(self.model)
        self.bt, self.support = self.g.bottle_pose(self.model)
        self.bottle = self.g.cylinder(self.model['reported']['bottle_diameter']/2, self.model['reported']['bottle_height'], int(self.model['sampling']['cylinder_segments']))
        self.base_rack = np.asarray(installation['desk_to_base']['matrix']) @ rigid_mm_deg(config['rack_xyz_mm'], config['rack_rpy_deg'])
        desk_rack = rigid_mm_deg(config['rack_xyz_mm'], config['rack_rpy_deg'])
        vertices = np.vstack([m.moved(desk_rack).vertices for m in self.rack.values()])
        if np.any(vertices[:, :2].min(0) < 0) or np.any(vertices[:, :2].max(0) > [installation['desk']['size_x_m'], installation['desk']['size_y_m']]) or abs(vertices[:, 2].min()) > 1e-9:
            lo, hi = vertices.min(0)*1000, vertices.max(0)*1000
            raise ValueError('rack must sit on and fit inside physical desktop: '
                             f'desk-frame rack bounds mm min={lo.round(3).tolist()}, '
                             f'max={hi.round(3).tolist()}; desktop XY mm='
                             f'{[installation["desk"]["size_x_m"]*1000, installation["desk"]["size_y_m"]*1000]}, '
                             'rack bottom Z must be 0; adjust rack_xyz_mm/rack_rpy_deg')
        self.links, self.joints, self.tcp = self.g.urdf_gripper(description)
        self.metrics = self.g.gripper_metrics(self.links, self.joints, self.tcp)
        engine = self.a.Proximity(backend, {**self.rack, **self.links, BOTTLE:self.bottle})
        try:
            self.axis, self.insertion_scan = self.a.choose_axis_x(engine, self.model, self.bt, self.links, self.joints, self.tcp, self.metrics)
        finally:
            engine.close()
        self.wrist, self.bottle_tcp = self.g.grasp(self.model, self.bt, self.metrics, self.tcp, self.axis)
        # Thomas permits the opposite finger orientation to avoid the half-turn
        # imposed by the old tool +Z = cap convention. Keep approach +X and the
        # physical gripping reference point unchanged; recheck actual geometry.
        tool_z_sign=config.get('grasp_tool_z_sign',1)
        if tool_z_sign not in (-1,1):raise ValueError('grasp_tool_z_sign must be +1 or -1')
        if tool_z_sign==-1:
            point=np.array([self.axis,self.metrics['inner_center_y_mm'],self.metrics['finger_z_mid_mm']])/1000
            center=self.wrist[:3,3]+self.wrist[:3,:3]@point
            self.wrist[:3,:3]=self.wrist[:3,:3]@np.diag([1.,-1.,-1.])
            self.wrist[:3,3]=center-self.wrist[:3,:3]@point
            self.bottle_tcp=np.linalg.inv(self.bt)@self.wrist@self.tcp
        if config.get('reverse_bottle_axis') is True:
            # Correct which physical end is the cap without moving the accepted
            # cylinder volume or any previously accepted gripper waypoint.
            reverse=np.eye(4)
            reverse[:3,:3]=np.diag([1.,-1.,-1.])
            reverse[2,3]=self.model['reported']['bottle_height']/1000
            self.bt=self.bt@reverse
            self.bottle_tcp=np.linalg.inv(self.bt)@self.wrist@self.tcp
        self.desk_spec = desk_collision_spec(installation)
        # Diagnostic physical box removes only the padding returned by VISION.
        # Never sent to MoveIt and never changes the installation safety margin.
        self.physical_desk_spec = copy.deepcopy(self.desk_spec)
        margin = self.desk_spec['safety_margin_m']
        self.physical_desk_spec['size_m'] = (np.array(self.desk_spec['size_m']) - [2*margin,2*margin,margin]).tolist()
        physical_pose = np.array(self.desk_spec['matrix'])
        physical_pose[:3,3] -= physical_pose[:3,2] * margin/2
        self.physical_desk_spec['matrix'] = physical_pose.tolist()
        self.camera_spec = camera_mount_collision_spec(installation)
        def box(size):
            half = np.array(size) * 500
            return self.g.box(-half, half)
        self.meshes = {**self.rack, **self.robot.meshes, BOTTLE:self.bottle,
                       'desk':box(self.desk_spec['size_m']),
                       'desk_physical':box(self.physical_desk_spec['size_m']),
                       'vision_assembly':box(self.camera_spec['size_m'])}
        self.engine = self.a.Proximity(backend, self.meshes)
        self.camera_local = self.g.pose(translation=self.camera_spec['center_m'])
        self.world = {name:self.base_rack for name in self.rack}
        self.world['desk'] = np.array(self.desk_spec['matrix'])
        self.world[BOTTLE] = self.base_rack @ self.bt
        self.open_q = (self.metrics['urdf_open_gap_mm'] - self.model['draft']['open_gap']) / 2000
        self.closed_q = (self.metrics['urdf_open_gap_mm'] - self.model['reported']['bottle_diameter']) / 2000

    def targets(self):
        w = self.base_rack @ self.wrist
        pre = w.copy(); pre[:3,3] -= w[:3,0] * self.model['draft']['approach_distance']/1000
        lift = w.copy(); lift[:3,3] += self.base_rack[:3,2] * self.model['draft']['lift_distance']/1000
        leave = lift.copy()
        if not self.config.get('upright_in_place',False):
            leave[:3,3] -= self.base_rack[:3,1] * self.model['draft']['front_exit_distance']/1000
        upright = leave.copy()
        sign=self.config.get('grasp_tool_z_sign',1)
        upright[:3,:3] = Rotation.from_euler('z', self.config['upright_yaw_deg'], degrees=True).as_matrix() @ np.diag([1.,sign,sign])
        if self.config.get('upright_quarter_turn') is True:
            cap_local=self.wrist[:3,:3].T@self.bt[:3,2]
            cap_world=leave[:3,:3]@cap_local
            z=np.array([0.,0.,1.])
            axis=np.cross(cap_world,z)
            angle=math.acos(float(np.clip(cap_world@z,-1.,1.)))
            if not math.isclose(angle,math.pi/2,abs_tol=1e-6):
                raise ValueError('quarter-turn preview expects an initially horizontal bottle')
            upright[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle).as_matrix()@leave[:3,:3]
        # Rotate about bottle grasp reference, not the wrist origin.
        center = (leave @ np.array([self.axis/1000,self.metrics['inner_center_y_mm']/1000,self.metrics['finger_z_mid_mm']/1000,1]))[:3]
        upright[:3,3] = center - upright[:3,:3] @ np.array([self.axis/1000,self.metrics['inner_center_y_mm']/1000,self.metrics['finger_z_mid_mm']/1000])
        offset=np.asarray(self.config.get('upright_offset_base_mm',[0,0,0]),dtype=float)
        if offset.shape!=(3,) or not np.isfinite(offset).all():
            raise ValueError('upright_offset_base_mm must contain three finite millimetre values')
        upright[:3,3]+=offset/1000
        return {k:v @ self.tcp for k,v in [('pregrasp',pre),('approach',w),('lift',lift),('exit',leave),('upright',upright)]}

    def check(self, q, attached=None, phase='initial'):
        ts = self.robot.fk(q)
        ts['vision_assembly'] = ts['Empty_Link6'] @ self.camera_local
        world = dict(self.world)
        if attached is not None:
            ts[BOTTLE] = ts['Empty_Link6'] @ attached
            del world[BOTTLE]
        pairs=[]
        for link, t in ts.items():
            if link not in self.meshes:
                continue
            for oid, ot in world.items():
                pairs.append((link,oid,t,ot))
        moving = [n for n in ts if n in self.meshes]
        for i, a in enumerate(moving):
            for b in moving[i+1:]:
                if frozenset((a,b)) in self.robot.disabled or set((a,b)) == {'vision_assembly','Empty_Link6'}:
                    continue
                pairs.append((a,b,ts[a],ts[b]))
        results=self.engine.pairs(pairs)
        records=[]
        for pair,result in zip(pairs,results):
            fixed_mount = (self.config.get('allow_fixed_base_desk_contact') is True
                           and set(pair[:2]) == {'base_link', 'desk'})
            allowed=fixed_mount and result['collision']
            finger_plate=(self.config.get('allow_finger_upper_plate_contact') is True
                          and phase in ('approach','close','lift')
                          and 'ganten_rack_upper_plate' in pair[:2]
                          and any(f in pair[:2] for f in FINGERS))
            if finger_plate:allowed=result['collision']
            if BOTTLE in pair[:2] and any(f in pair[:2] for f in FINGERS) and phase in ('close','lift','exit','upright'):
                name=next(n for n in pair[:2] if n in FINGERS)
                gap=self.metrics['urdf_open_gap_mm']-2000*q[6]
                allowed=result['collision'] and self.a.bottle_contact_allowed(result,name,ts['Empty_Link6'],gap,self.model,self.metrics)
            if (attached is not None and BOTTLE in pair[:2] and any(n in SUPPORT for n in pair[:2])
                    and phase=='lift' and np.allclose(ts[BOTTLE],self.world[BOTTLE],atol=1e-10,rtol=0)
                    and not result['contained']):
                # Only the independently solved original support pose, never a displaced penetration.
                allowed=result['collision']
            row={'pair':pair[:2],**result,'allowed_contact':bool(allowed),'blocked':bool(result['collision'] and not allowed)}
            if fixed_mount:row['contact_rule']='operator_authorized_fixed_base_desk'
            if finger_plate:row['contact_rule']='operator_authorized_finger_upper_plate'
            if result['collision'] and 'desk' in pair[:2]:
                physical=self.engine.pairs([(pair[0],'desk_physical',pair[2],np.array(self.physical_desk_spec['matrix']))])[0]
                row['physical_desk']=physical
                row['classification']='physical_intersection_or_surface_contact' if physical['collision'] else 'margin_only'
            records.append(row)
        return {'bounds_violations':self.robot.bounds(q),'blocked':bool(self.robot.bounds(q) or any(r['blocked'] for r in records)), 'pairs':records}

    def metadata(self):
        return {'status':'DRAFT_SIMULATION','config':self.config,'model':self.model,'T_base_rack':self.base_rack.tolist(),
                'T_base_bottle':self.world[BOTTLE].tolist(),'T_bottle_tcp':self.bottle_tcp.tolist(),
                'targets_base_tcp':{k:v.tolist() for k,v in self.targets().items()},
                'desk':self.desk_spec,'camera':self.camera_spec,'open_q_m':self.open_q,'closed_q_m':self.closed_q,
                'axis_x_mm':self.axis,'support_points_rack_m':self.support.tolist(),'post_notes':self.post_notes,
                'owned_world_ids':list(self.world),'owned_attached_ids':['vision_assembly',BOTTLE]}

    def close(self):
        self.engine.close()
