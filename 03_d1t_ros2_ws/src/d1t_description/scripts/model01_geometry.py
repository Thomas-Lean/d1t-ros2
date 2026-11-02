#!/usr/bin/env python3
"""MODEL-01 geometry. Input mm -> mesh/poses m once; no ROS or motion commands.

The rack is a set of closed solids, NOT a solid bounding box. Extruded plate
thickness is vertical (DRAFT); side lips and sockets are explicit approximations.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import math
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import yaml


@dataclass
class Mesh:
    vertices: np.ndarray
    faces: np.ndarray

    def moved(self, transform):
        return Mesh(self.vertices @ transform[:3, :3].T + transform[:3, 3], self.faces)

    def payload(self):
        edges = np.concatenate([self.faces[:, [0, 1]], self.faces[:, [1, 2]],
                                self.faces[:, [2, 0]]])
        graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                           shape=(len(self.vertices), len(self.vertices)))
        count, labels = connected_components(graph, directed=False)
        refs = [int(np.flatnonzero(labels == i)[0]) for i in range(count)]
        return {'vertices': self.vertices.tolist(), 'faces': self.faces.tolist(),
                'representatives': refs}


def pose(rotation=None, translation=None):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    if translation is not None:
        t[:3, 3] = translation
    return t


def quaternion(t):
    return Rotation.from_matrix(t[:3, :3]).as_quat().tolist()  # xyzw


def load_config(path):
    c = yaml.safe_load(Path(path).read_text())
    validate(c)
    return c


def validate(c):
    r, d, s = c['reported'], c['draft'], c['sampling']
    for section in [r, s]:
        for key, value in section.items():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
    for key, value in d.items():
        if isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError(f'{key} must be finite')
    if not 0 < r['grip_h'] < r['bottle_height']:
        raise ValueError('grip_h outside bottle')
    if not r['lower_bottom'] < r['upper_bottom'] < r['rack_height']:
        raise ValueError('invalid plate heights')
    if r['upper_bottom'] + r['slot_depth'] > r['rack_height']:
        raise ValueError('upper crest exceeds reported rack height')
    if r['edge_thickness'] < r['plate_thickness']:
        raise ValueError('edge must be at least plate thickness')
    if len(d['upper_centers']) != 3 or len(d['lower_centers']) != 4:
        raise ValueError('requires three upper and four lower slots')
    for centers in [d['upper_centers'], d['lower_centers']]:
        if not all(0 < a < r['rack_width'] for a in centers) or np.any(np.diff(centers) <= 0):
            raise ValueError('slot centers must be ordered within rack')
        if np.min(np.diff(centers)) < r['slot_width']:
            raise ValueError('slot width exceeds pitch; overlapping slot hypotheses')
    if d['profile'] not in ['cosine', 'v']:
        raise ValueError('profile must be cosine or v')
    if d['selected_slot'] not in range(3):
        raise ValueError('selected_slot must be 0, 1, or 2')
    if not 0 < d['open_gap'] or d['backstop_gap'] < 0:
        raise ValueError('invalid gripper gaps')
    if int(s['cylinder_segments']) != s['cylinder_segments'] or s['cylinder_segments'] < 32:
        raise ValueError('cylinder_segments must be integer >=32')
    for key in ['edge_band', 'lower_slot_depth', 'approach_distance', 'lift_distance',
                'front_exit_distance']:
        if d[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if 2 * d['edge_band'] >= r['rack_depth']:
        raise ValueError('edge bands cover full plate')
    if s['profile_step'] > r['slot_width'] / 10:
        raise ValueError('profile sampling too coarse')
    for x in [r['post_front_x'], r['post_front_x'] + r['post_dx']]:
        for y in [r['post_front_y'], r['post_front_y'] + r['post_dy']]:
            rad = r['socket_diameter'] / 2
            if not rad < x < r['rack_width'] - rad or not rad < y < r['rack_depth'] - rad:
                raise ValueError('socket outside rack envelope')


def profile(c, x, layer):
    r, d = c['reported'], c['draft']
    centers = np.asarray(d[layer + '_centers'])
    distance = np.min(np.abs(np.asarray(x)[..., None] - centers), axis=-1)
    u = np.minimum(distance / (r['slot_width'] / 2), 1)
    depth = r['slot_depth'] if layer == 'upper' else d['lower_slot_depth']
    shape = (1 - np.cos(np.pi * u)) / 2 if d['profile'] == 'cosine' else u
    return r[layer + '_bottom'] + depth * shape


def profile_x(c):
    r, d = c['reported'], c['draft']
    xs = list(np.linspace(0, r['rack_width'], math.ceil(r['rack_width'] / c['sampling']['profile_step']) + 1))
    for centers in [d['upper_centers'], d['lower_centers']]:
        for center in centers:
            xs.extend([center, max(0, center-r['slot_width']/2),
                       min(r['rack_width'], center+r['slot_width']/2)])
    return np.unique(xs)


def sheet(xs, top, y0, y1, thickness):
    # Four vertices per cross section; end caps + sides -> watertight solid.
    v = np.array([[x, y, z] for x, h in zip(xs, top)
                  for y, z in [(y0, h), (y1, h), (y0, h-thickness), (y1, h-thickness)]]) / 1000
    f = []
    for i in range(len(xs)-1):
        a, b = 4*i, 4*(i+1)
        f.extend([[a, b, b+1], [a, b+1, a+1], [a+2, a+3, b+3], [a+2, b+3, b+2],
                  [a, a+2, b+2], [a, b+2, b], [a+1, b+1, b+3], [a+1, b+3, a+3]])
    b = 4*(len(xs)-1)
    f.extend([[0, 1, 3], [0, 3, 2], [b, b+2, b+3], [b, b+3, b+1]])
    return Mesh(v, np.array(f, dtype=int))


def box(lo, hi):
    lo, hi = np.asarray(lo), np.asarray(hi)
    if np.any(hi <= lo):
        raise ValueError('box must have positive dimensions')
    return sheet([lo[0], hi[0]], [hi[2], hi[2]], lo[1], hi[1], hi[2]-lo[2])


def cylinder(radius, length, segments=64):
    a = np.arange(segments) * 2 * np.pi / segments
    ring = np.column_stack([radius*np.cos(a), radius*np.sin(a), np.zeros(segments)])
    v = np.vstack([ring, ring+[0, 0, length], [0, 0, 0], [0, 0, length]]) / 1000
    f = []
    for i in range(segments):
        j = (i+1) % segments
        f.extend([[i, j, j+segments], [i, j+segments, i+segments],
                  [2*segments, j, i], [2*segments+1, i+segments, j+segments]])
    return Mesh(v, np.array(f, dtype=int))


def rack(c):
    validate(c)
    r, d = c['reported'], c['draft']
    xs = profile_x(c)
    objects = {}
    for layer in ['upper', 'lower']:
        top = profile(c, xs, layer)
        name = 'ganten_rack_' + layer
        objects[name + '_plate'] = sheet(xs, top, 0, r['rack_depth'], r['plate_thickness'])
        for tag, y0, y1 in [('front', 0, d['edge_band']),
                            ('back', r['rack_depth']-d['edge_band'], r['rack_depth'])]:
            # Thickening below the measured top, not a measured extra raised rim.
            objects[name+'_'+tag+'_edge'] = sheet(xs, top, y0, y1, r['edge_thickness'])
        for tag, x0, x1 in [('left', 0, r['edge_thickness']),
                            ('right', r['rack_width']-r['edge_thickness'], r['rack_width'])]:
            # DRAFT terminal lip; upper top satisfies the total-height report.
            h = r['rack_height'] if layer == 'upper' else r['lower_bottom']+d['lower_slot_depth']
            z0 = float(np.min(profile(c, [x0, x1], layer))) - r['plate_thickness']
            objects[name+'_'+tag+'_lip'] = box([x0, 0, z0], [x1, r['rack_depth'], h])
    post_notes = []
    for i, x in enumerate([r['post_front_x'], r['post_front_x']+r['post_dx']]):
        for j, y in enumerate([r['post_front_y'], r['post_front_y']+r['post_dy']]):
            bottom = float(profile(c, x, 'lower'))
            top = float(profile(c, x, 'upper')) - r['plate_thickness']
            exposed = top-bottom-2*r['socket_visible']
            if exposed <= 0:
                raise ValueError('connectors leave no exposed post')
            name = f'ganten_rack_post_{i}{j}'
            objects[name] = cylinder(r['post_diameter']/2, top-bottom).moved(pose(translation=np.array([x,y,bottom])/1000))
            for tag, z in [('lower', bottom), ('upper', top-r['socket_visible'])]:
                objects[name+'_'+tag+'_socket'] = cylinder(r['socket_diameter']/2, r['socket_visible']).moved(pose(translation=np.array([x,y,z])/1000))
            post_notes.append({'id': name, 'modeled_exposed_mm': exposed,
                               'reported_exposed_mm': r['post_exposed']})
    for i, x in enumerate(d['lower_centers']):
        for j, y in enumerate([r['post_front_y'], r['post_front_y']+r['post_dy']]):
            length = float(profile(c, x, 'lower'))-r['plate_thickness']
            objects[f'ganten_rack_foot_{i}{j}'] = cylinder(r['socket_diameter']/2, length).moved(pose(translation=np.array([x,y,0])/1000))
    return objects, post_notes


def bottle_pose(c):
    """Lowest non-penetrating pose of the generated convex cylinder polyhedron.

    For each linear plate strip maximize (plate height - bottle point Z), with
    the point constrained inside the finite cylinder and the strip's X/Y range.
    Linear programming covers faces and end caps, not only sampled vertices.
    This is geometric support, not a stability/friction solve.
    """
    r, d, s = c['reported'], c['draft'], c['sampling']
    base_rotation = Rotation.from_euler('x', np.pi/2).as_matrix()  # cap -> rack -Y
    rot = Rotation.from_euler('zx', [d['bottle_yaw'], d['bottle_pitch']]).as_matrix() @ base_rotation
    angles = np.arange(int(s['cylinder_segments']))*2*np.pi/s['cylinder_segments']
    ring = np.column_stack([r['bottle_diameter']/2*np.cos(angles),
                            r['bottle_diameter']/2*np.sin(angles),np.zeros(len(angles))])
    local = np.vstack([ring,ring+[0,0,r['bottle_height']]])
    x = d['upper_centers'][d['selected_slot']]+d['bottle_offset_x']
    points = local @ rot.T + [x, d['bottle_bottom_y'], 0]
    hull = ConvexHull(points).equations
    xs = profile_x(c)
    zs = profile(c,xs,'upper')
    strips = list(zip(xs[:-1],xs[1:],zs[:-1],zs[1:]))
    strips.extend([(0,r['edge_thickness'],r['rack_height'],r['rack_height']),
                   (r['rack_width']-r['edge_thickness'],r['rack_width'],r['rack_height'],r['rack_height'])])
    solutions=[]
    for a,b,za,zb in strips:
        if b<points[:,0].min() or a>points[:,0].max():continue
        slope=(zb-za)/(b-a)
        solution=linprog([-slope,0,1],A_ub=hull[:,:3],b_ub=-hull[:,3],
                         bounds=[(a,b),(0,r['rack_depth']),(None,None)],method='highs')
        if solution.success:solutions.append((za-slope*a-solution.fun,solution.x))
        elif solution.status!=2:raise ValueError('support linear program failed: '+solution.message)
    if not solutions:raise ValueError('bottle has no support over the finite plate')
    z=max(h for h,_ in solutions)
    contacts=np.array([p for h,p in solutions if h>=z-s['contact_tolerance']])
    contacts[:,2] += z
    return pose(rot, np.array([x, d['bottle_bottom_y'], z])/1000), contacts/1000


def read_stl(path):
    b = Path(path).read_bytes()
    n = struct.unpack('<I', b[80:84])[0]
    if len(b) != 84 + 50*n:
        raise ValueError(f'expected binary STL: {path}')
    tri = np.frombuffer(b[84:], dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]))['v'].astype(float)
    vertices, indices = np.unique(tri.reshape(-1,3), axis=0, return_inverse=True)
    return Mesh(vertices, indices.reshape(-1,3))


def write_stl(mesh, path):
    tri = mesh.vertices[mesh.faces]
    normal = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
    norms = np.linalg.norm(normal, axis=1)
    if np.any(norms < 1e-15):
        raise ValueError('degenerate triangle')
    normal /= norms[:,None]
    data = np.zeros(len(tri), dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]))
    data['normal'], data['v'] = normal, tri
    Path(path).write_bytes(b'MODEL-01; vertex units metres'.ljust(80,b' ') + struct.pack('<I',len(tri)) + data.tobytes())


def urdf_gripper(description_dir):
    p = Path(description_dir)
    root = ET.parse(p/'urdf/d1t.urdf').getroot()
    def origin(element):
        if element is None:
            return np.eye(4)
        xyz = [float(x) for x in element.get('xyz','0 0 0').split()]
        rpy = [float(x) for x in element.get('rpy','0 0 0').split()]
        return pose(Rotation.from_euler('xyz',rpy).as_matrix(), xyz)
    links = {}
    for name in ['Empty_Link_L','Empty_Link_R','Empty_Link6']:
        col = root.find(f"link[@name='{name}']/collision")
        m = col.find('geometry/mesh')
        local = read_stl(p / m.get('filename').split('package://d1t_description/')[1])
        local.vertices *= np.array([float(x) for x in m.get('scale','1 1 1').split()])
        links[name] = local.moved(origin(col.find('origin')))
    joints = {}
    for name in ['Joint_L','Joint_R']:
        e = root.find(f"joint[@name='{name}']")
        joints[name] = {'origin':origin(e.find('origin')), 'axis':np.array([float(x) for x in e.find('axis').get('xyz').split()]),
                        'limit':[float(e.find('limit').get(k)) for k in ['lower','upper']]}
    tcp = origin(root.find("joint[@name='tcp_joint']/origin"))
    return links, joints, tcp


def finger_transform(joints, name, q):
    j = joints['Joint_'+name[-1]]
    if not j['limit'][0]-1e-10 <= q <= j['limit'][1]+1e-10:
        raise ValueError('gripper gap requires joint value outside existing URDF limits')
    return j['origin'] @ pose(translation=j['axis']*q)


def gripper_metrics(links, joints, tcp):
    left = links['Empty_Link_L'].moved(finger_transform(joints,'Empty_Link_L',0)).vertices
    right = links['Empty_Link_R'].moved(finger_transform(joints,'Empty_Link_R',0)).vertices
    # Inner straight contact faces from actual meshes; q=0 is the model open state.
    yl, yr = left[:,1].max(), right[:,1].min()
    return {'urdf_open_gap_mm':float((yr-yl)*1000), 'inner_center_y_mm':float((yr+yl)*500),
            'finger_z_mid_mm':float((left[:,2].min()+left[:,2].max())*500),
            'finger_tip_x_mm':float(left[:,0].max()*1000),
            'link_bounds_mm':{n:[(m.vertices.min(0)*1000).tolist(),(m.vertices.max(0)*1000).tolist()] for n,m in links.items()},
            'tcp_xyz_mm':(tcp[:3,3]*1000).tolist()}


def grasp(c, bottle_t, metrics, tcp, axis_x_mm):
    # tool +Z = bottle axis; tool +X approaches from above, optionally rolled
    # around that axis. This is full SO(3), never a horizontal yaw shortcut.
    z = bottle_t[:3,2]
    x = np.array([0.,0.,-1.])
    x -= z*np.dot(x,z)
    if np.linalg.norm(x) < 1e-8:
        raise ValueError('top approach undefined for vertical bottle')
    x /= np.linalg.norm(x)
    x = Rotation.from_rotvec(z*c['draft']['tool_roll']).apply(x)
    rot = np.column_stack([x,np.cross(z,x),z])
    center = bottle_t[:3,3]+z*c['reported']['grip_h']/1000
    axis_tool = np.array([axis_x_mm,metrics['inner_center_y_mm'],metrics['finger_z_mid_mm']])/1000
    wrist = pose(rot,center-rot@axis_tool)
    tcp_world = wrist@tcp
    bottle_to_tcp = np.linalg.inv(bottle_t)@tcp_world
    return wrist, bottle_to_tcp


def export(objects, out):
    out = Path(out)
    out.mkdir(parents=True,exist_ok=True)
    for name, mesh in objects.items():
        write_stl(mesh, out/(name+'.stl'))
    (out/'objects.json').write_text(json.dumps({'frame_id':'rack_frame','vertex_units':'m','objects':list(objects)},indent=2)+'\n')
