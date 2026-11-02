"""ROS2 wrappers. Capture only subscribes and calls read-only FK/parameter services.

No action clients, robot command publishers or driver motion services exist here.
ROS2 imports deliberately stay outside the calculation modules.
"""
from collections import deque
from copy import deepcopy
import json
import threading
import uuid
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PointStamped, TransformStamped
from moveit_msgs.srv import GetPositionFK
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from .board import detect_board
from .calibration import load_calibration
from .geometry import (ValidationError, apply, decode_image, pose_matrix, read_transform, transform_record)
from .runtime import (choose_inputs, mount_transform, quaternion_xyzw, validate_config,
                      check_point_joint_bracket)
from .samples import (CAMERA_STREAMS, digest, read_json, save_sample, stamp, validate_timing,
                      validate_sample, joint_units)


def ros_ns(value):
    return value.sec*1_000_000_000+value.nanosec


def tf_matrix(value):
    t, q = value.transform.translation, value.transform.rotation
    return pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])


def complete(future, timeout):
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    if not event.wait(timeout):
        future.cancel()
        raise ValidationError('read-only service response timed out')
    result = future.result()
    if result is None:
        raise ValidationError('read-only service returned no response')
    return result


def verify_actual_source(self):
    pubs = self.get_publishers_info_by_topic(self.cfg['topics']['joint_state'])
    names = [(p.node_namespace.rstrip('/')+'/'+p.node_name) for p in pubs]
    expected = self.cfg['joint_state_publisher_node']
    if names != [expected]:
        raise ValidationError(f'expected one actual JointState publisher {expected}, observed {names}')
    if not self.source_query.service_is_ready():
        raise ValidationError('JointState source parameter service unavailable')
    request = GetParameters.Request()
    request.names = ['source']
    result = complete(self.source_query.call_async(request),self.cfg['fk_timeout_s'])
    if len(result.values) != 1 or result.values[0].string_value != 'feedback':
        raise ValidationError('JointState publisher source must be feedback, never static/demo')


class ConfigNode(Node):
    def __init__(self, name):
        super().__init__(name)
        path = self.declare_parameter('config_path', '').value
        self.cfg = validate_config(read_json(path))
        if self.get_parameter('use_sim_time').value:
            raise ValidationError('real capture/TF nodes reject use_sim_time; synthetic tests run offline')

    def calibration(self):
        path = self.declare_parameter('calibration_file', '').value
        result = load_calibration(path)  # never an allow-test option in a ROS node
        contract = result['contract']
        for name in ('device_id', 'mount_id', 'optical_frame', 'board', 'image_geometry'):
            if self.cfg[name] != contract[name]:
                raise ValidationError(f'runtime/calibration {name} mismatch')
        self.get_logger().warn('Calibration is UNVALIDATED_REAL_CALIBRATION: read-only observation, no grasp acceptance.')
        return result


class CaptureNode(ConfigNode):
    def __init__(self):
        super().__init__('vision_capture')
        self.buffers = {s: deque(maxlen=120 if s == 'joint_state' else 60)
                        for s in (*CAMERA_STREAMS, 'joint_state')}
        self.buffer_lock, self.capture_lock = threading.Lock(), threading.Lock()
        self.last_saved_ns = 0
        self.io_group = ReentrantCallbackGroup()
        self.fk = self.create_client(GetPositionFK, self.cfg['fk_service'], callback_group=self.io_group)
        publisher = self.cfg['joint_state_publisher_node'].rstrip('/')
        self.source_query = self.create_client(GetParameters, publisher+'/get_parameters', callback_group=self.io_group)
        for name in CAMERA_STREAMS:
            kind = CameraInfo if name.endswith('info') else Image
            self.create_subscription(kind, self.cfg['topics'][name],
                                     lambda msg, s=name: self.on_camera(s,msg), qos_profile_sensor_data,
                                     callback_group=self.io_group)
        self.create_subscription(JointState, self.cfg['topics']['joint_state'], self.on_joint,
                                 qos_profile_sensor_data, callback_group=self.io_group)
        self.create_service(Trigger, '~/capture_sample', self.capture)
        self.get_logger().info('Read-only capture ready; request captures a stopped pose, it does not move the arm.')

    def header(self, name, msg):
        return {'stamp_ns': stamp(ros_ns(msg.header.stamp)),
                'received_ns': self.get_clock().now().nanoseconds,
                'frame_id': msg.header.frame_id, 'stamp_semantics': self.cfg['stamp_semantics'][name]}

    def enqueue(self, name, meta, array=None):
        with self.buffer_lock:
            entries = self.buffers[name]
            if entries and meta['stamp_ns'] <= entries[-1][0]['stamp_ns']:
                entries.clear()
                raise ValidationError(f'{name}: duplicate/backwards header timestamp; buffer cleared')
            entries.append((meta,array))
            cutoff = meta['received_ns']-self.cfg['pairing_policy']['max_age_ms']*1e6
            while entries and entries[0][0]['received_ns'] < cutoff:
                entries.popleft()

    def on_camera(self, name, msg):
        try:
            meta = self.header(name,msg)
            if name.endswith('info'):
                meta.update(width=msg.width,height=msg.height,k=list(msg.k),d=list(msg.d),r=list(msg.r),p=list(msg.p),
                            distortion_model=msg.distortion_model,binning_x=msg.binning_x,binning_y=msg.binning_y,
                            roi={key:getattr(msg.roi,key) for key in ('x_offset','y_offset','width','height','do_rectify')})
                array = None
            else:
                meta.update(encoding=msg.encoding,width=msg.width,height=msg.height,step=msg.step,is_bigendian=bool(msg.is_bigendian))
                array = decode_image(msg.data,msg.width,msg.height,msg.step,msg.encoding,msg.is_bigendian)
                if name == 'depth':
                    if msg.encoding != self.cfg['depth']['encoding']:
                        raise ValidationError('live depth encoding differs from confirmed config')
                    meta.update(self.cfg['depth'])
            self.enqueue(name,meta,array)
        except (ValidationError,ValueError) as exc:
            with self.buffer_lock:
                self.buffers[name].clear()
            self.get_logger().error(f'{name} rejected: {exc}')

    def on_joint(self, msg):
        try:
            meta = self.header('joint_state',msg)
            meta.update(names=list(msg.name),position=list(msg.position),position_units=joint_units(msg.name),
                        velocity=list(msg.velocity),effort=list(msg.effort))
            self.enqueue('joint_state',meta)
        except ValidationError as exc:
            self.get_logger().error(f'joint state rejected: {exc}')

    def capture(self, request, response):
        del request
        if not self.capture_lock.acquire(blocking=False):
            response.success, response.message = False, 'capture already in progress'
            return response
        try:
            verify_actual_source(self)
            with self.buffer_lock:
                # Buffers are append-only immutable measurement records; selection copies arrays.
                selected, color, depth = choose_inputs(self.buffers,self.cfg,self.get_clock().now().nanoseconds,self.last_saved_ns)
            meta = deepcopy(selected)
            for key in ('session_id','device_id','mount_id','board_fixed_evidence','joint_source_evidence',
                        'optical_frame','image_geometry','board','clock','pairing_policy'):
                meta[key] = deepcopy(self.cfg[key])
            meta.update(schema='d1t_vision.sample.v1',test_only=False,sample_id=str(uuid.uuid4()),
                        base_frame='base_link',ee_frame='Empty_Link6',topics=deepcopy(self.cfg['topics']))
            joint = meta['joint_state']
            meta['fk'] = {'stamp_ns':joint['stamp_ns'],'joint_state_sha256':digest(joint),
                          'source':self.cfg['fk_service']+' GetPositionFK with this explicit JointState'}
            validate_timing(meta)  # reject stale/moving/mismatched inputs before any FK or disk write
            if not self.fk.service_is_ready():
                raise ValidationError('read-only FK service unavailable')
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = 'base_link'
            fk_request.header.stamp = Time(nanoseconds=joint['stamp_ns']).to_msg()
            fk_request.fk_link_names = ['Empty_Link6']
            state = fk_request.robot_state.joint_state
            state.header.stamp = fk_request.header.stamp
            state.name, state.position = joint['names'], joint['position']
            fk_request.robot_state.is_diff = False
            fk_result = complete(self.fk.call_async(fk_request),self.cfg['fk_timeout_s'])
            if (fk_result.error_code.val != 1 or list(fk_result.fk_link_names) != ['Empty_Link6']
                    or len(fk_result.pose_stamped) != 1 or fk_result.pose_stamped[0].header.frame_id != 'base_link'):
                raise ValidationError('FK returned failure/wrong frame/link')
            value = fk_result.pose_stamped[0]
            p,q = value.pose.position,value.pose.orientation
            matrix = pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
            meta['fk']['transform'] = transform_record(matrix,'Empty_Link6','base_link')
            meta['fk']['response_header_stamp_ns'] = ros_ns(value.header.stamp)
            meta['fk']['response_received_ros_ns'] = self.get_clock().now().nanoseconds
            # The FK response timestamp is retained verbatim; the actual FK input timestamp is above.
            meta['observation'] = detect_board(color,meta['color']['encoding'],meta['color_info'],
                                               meta['image_geometry'],meta['board'],self.cfg['max_reprojection_rms_px'])
            validate_sample(meta,color,depth)
            path = save_sample(self.cfg['output_directory'],meta,color,depth)
            self.last_saved_ns = meta['color']['stamp_ns']
            response.success,response.message = True,str(path)
            self.get_logger().info(f'SAVED {path} image_stamp_ns={self.last_saved_ns}')
        except Exception as exc:
            response.success,response.message = False,f'{type(exc).__name__}: {exc}'
            self.get_logger().error('CAPTURE_REJECTED '+response.message)
        finally:
            self.capture_lock.release()
        return response


class PointNode(ConfigNode):
    def __init__(self):
        super().__init__('vision_point_to_base')
        self.result = self.calibration()
        self.x = read_transform(self.result['transform'],self.cfg['optical_frame'],'Empty_Link6')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pub = self.create_publisher(PointStamped,'~/base_point',10)
        self.create_subscription(PointStamped,'~/camera_point',self.on_point,10)
        self.joints = deque(maxlen=120)
        self.io_group = ReentrantCallbackGroup()
        self.source_query = self.create_client(GetParameters,self.cfg['joint_state_publisher_node']+'/get_parameters',callback_group=self.io_group)
        self.create_subscription(JointState,self.cfg['topics']['joint_state'],self.on_joint,qos_profile_sensor_data)

    def on_joint(self, msg):
        t = ros_ns(msg.header.stamp)
        if t <= 0 or (self.joints and t <= self.joints[-1]['stamp_ns']):
            self.joints.clear()
            return
        self.joints.append({'stamp_ns':t,'received_ns':self.get_clock().now().nanoseconds,
                            'stamp_semantics':self.cfg['stamp_semantics']['joint_state'],
                            'names':list(msg.name),'position':list(msg.position),'position_units':joint_units(msg.name)})

    def on_point(self, msg):
        try:
            t = stamp(ros_ns(msg.header.stamp))
            age = self.get_clock().now().nanoseconds-t
            if msg.header.frame_id != self.cfg['optical_frame'] or not 0 <= age <= self.cfg['pairing_policy']['max_age_ms']*1e6:
                raise ValidationError('point has wrong optical frame, zero/future/stale image stamp')
            verify_actual_source(self)
            check_point_joint_bracket(list(self.joints),t,self.get_clock().now().nanoseconds,self.cfg['pairing_policy'])
            # Exact requested time; never Time() / latest-TF fallback. No camera TF attachment needed.
            g = tf_matrix(self.tf.lookup_transform('base_link','Empty_Link6',Time(nanoseconds=t)))
            out = apply(g@self.x,[msg.point.x,msg.point.y,msg.point.z])
            if msg.point.z <= 0:
                raise ValidationError('camera point must be in front of optical plane')
            value = PointStamped()
            value.header.stamp,value.header.frame_id = msg.header.stamp,'base_link'
            value.point.x,value.point.y,value.point.z = [float(x) for x in out]
            self.pub.publish(value)
        except (ValidationError,TransformException) as exc:
            self.get_logger().error(f'POINT_REJECTED (no output): {exc}')


class MountNode(ConfigNode):
    def __init__(self):
        super().__init__('vision_camera_mount')
        self.result = self.calibration()
        self.root = self.declare_parameter('camera_root_frame','').value
        self.edges = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.broadcaster = StaticTransformBroadcaster(self)
        static_qos = QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TFMessage,'/tf_static',self.observe,static_qos)
        self.create_subscription(TFMessage,'/tf',self.observe,100)
        self.timer = self.create_timer(5.,self.attach_once)

    def observe(self,msg,info):
        publisher = bytes(info.publisher_gid).hex()
        for t in msg.transforms:
            self.edges.setdefault(t.child_frame_id,set()).add((t.header.frame_id,publisher))

    def attach_once(self):
        self.timer.cancel()
        try:
            internal = self.tf.lookup_transform(self.root,self.cfg['optical_frame'],Time())
            # The driver's internal optical chain must be static.
            if ros_ns(internal.header.stamp) != 0:
                raise ValidationError('camera internal root-to-optical transform must be static')
            record = mount_transform(self.result,self.root,tf_matrix(internal),self.edges)
            t = read_transform(record,self.root,'Empty_Link6')
            value = TransformStamped()
            value.header.stamp,value.header.frame_id = self.get_clock().now().to_msg(),'Empty_Link6'
            value.child_frame_id = self.root
            q = quaternion_xyzw(t)
            value.transform.translation.x,value.transform.translation.y,value.transform.translation.z = [float(x) for x in t[:3,3]]
            value.transform.rotation.x,value.transform.rotation.y,value.transform.rotation.z,value.transform.rotation.w = [float(x) for x in q]
            self.broadcaster.sendTransform(value)
            self.get_logger().info('Published one mount TF to driver root: '+json.dumps(record))
        except (ValidationError,TransformException) as exc:
            self.get_logger().error(f'TF_ATTACH_REJECTED: {exc}; restart only after graph/config correction')


def run(node_type,args=None):
    rclpy.init(args=args)
    node,executor = None,None
    try:
        node=node_type()
        executor=MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def capture_main(args=None):
    run(CaptureNode,args)


def point_main(args=None):
    run(PointNode,args)


def mount_main(args=None):
    run(MountNode,args)
