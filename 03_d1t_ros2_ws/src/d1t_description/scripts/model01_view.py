#!/usr/bin/env python3
"""Display exported MODEL-01 meshes in an isolated rack_frame (domain 7 only).

Publishes visualization markers, isolated scene messages and static local frames.
Never applies a scene to move_group, and has no movement/service/action clients.
"""
import argparse
import json
import os
from pathlib import Path
import time

from model01_geometry import read_stl


def scene_objects(directory):
    from geometry_msgs.msg import Point, Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh, MeshTriangle
    objects=[]
    for file in sorted(Path(directory).glob('*.stl')):
        mesh=read_stl(file)
        msg=Mesh()
        msg.vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in mesh.vertices]
        msg.triangles=[MeshTriangle(vertex_indices=f.tolist()) for f in mesh.faces]
        obj=CollisionObject()
        obj.header.frame_id='rack_frame';obj.id=file.stem
        obj.operation=CollisionObject.ADD;obj.meshes=[msg]
        p=Pose();p.orientation.w=1.;obj.mesh_poses=[p]
        objects.append(obj)
    return objects


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--duration',type=float,default=0,help='seconds; 0 until interrupted')
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='7':
        raise SystemExit('MODEL-01 view requires project simulation entry and ROS_DOMAIN_ID=7')
    if os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp' or not os.environ.get('CYCLONEDDS_URI','').endswith('/cyclonedds_sim.xml'):
        raise SystemExit('source the existing project simulation environment first')
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from visualization_msgs.msg import Marker,MarkerArray
    from geometry_msgs.msg import Point,TransformStamped
    from moveit_msgs.msg import PlanningScene
    from tf2_ros import StaticTransformBroadcaster
    rclpy.init();node=rclpy.create_node('model01_local_view')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(MarkerArray,'/model01/markers',qos)
    scene_pub=node.create_publisher(PlanningScene,'/model01/scene',qos)
    data=json.loads(args.result.read_text())
    objects=scene_objects(args.result.parent/'meshes')
    markers=MarkerArray()
    for i,obj in enumerate(objects):
        m=Marker();m.header.frame_id='rack_frame';m.ns='model01';m.id=i
        m.type=Marker.TRIANGLE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1.
        m.scale.x=m.scale.y=m.scale.z=1.
        if obj.id=='ganten_348_bottle': color=(0.12,0.45,0.90,0.85)
        elif obj.id.startswith('Empty_'): color=(0.95,0.43,0.10,1.0)
        elif 'post' in obj.id: color=(0.65,0.68,0.72,1.0)
        else: color=(0.35,0.72,0.72,0.7)
        m.color.r,m.color.g,m.color.b,m.color.a=color
        mesh=obj.meshes[0]
        m.points=[mesh.vertices[index] for f in mesh.triangles for index in f.vertex_indices]
        markers.markers.append(m)
    ref=Marker();ref.header.frame_id='rack_frame';ref.ns='model01';ref.id=len(objects)
    ref.type=Marker.SPHERE;ref.pose.orientation.w=1.
    ref.pose.position.x,ref.pose.position.y,ref.pose.position.z=data['grip_reference_m']
    ref.scale.x=ref.scale.y=ref.scale.z=.004;ref.color.r=1.;ref.color.g=.1;ref.color.a=1.
    markers.markers.append(ref)
    for axis,color in enumerate([(1.,0.,0.),(0.,1.,0.),(0.,0.,1.)]):
        m=Marker();m.header.frame_id='rack_frame';m.ns='model01_axes';m.id=axis
        m.type=Marker.ARROW;m.pose.orientation.w=1.;m.scale.x=.002;m.scale.y=.005;m.scale.z=.008
        m.color.r,m.color.g,m.color.b=color;m.color.a=1.
        end=[0.,0.,0.];end[axis]=.06
        m.points=[Point(),Point(x=end[0],y=end[1],z=end[2])];markers.markers.append(m)
    scene=PlanningScene();scene.name='MODEL-01 DRAFT local display only';scene.is_diff=True
    scene.world.collision_objects=[o for o in objects if o.id.startswith('ganten_')]
    tf=StaticTransformBroadcaster(node)
    t=TransformStamped();t.header.frame_id='rack_frame';t.child_frame_id='model01_bottle_frame'
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=data['bottle_bottom_m']
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=data['bottle_quaternion_xyzw']
    tf.sendTransform(t)
    def publish():
        stamp=node.get_clock().now().to_msg()
        for m in markers.markers:m.header.stamp=stamp
        pub.publish(markers);scene_pub.publish(scene)
    publish();node.create_timer(1.,publish)
    print(json.dumps({'objects':len(objects),'markers':len(markers.markers),
                      'domain':os.environ['ROS_DOMAIN_ID'],'frame':'rack_frame',
                      'topics':['/model01/markers','/model01/scene','/tf_static']}),flush=True)
    start=time.monotonic()
    try:
        while rclpy.ok() and (args.duration<=0 or time.monotonic()-start<args.duration):
            rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
