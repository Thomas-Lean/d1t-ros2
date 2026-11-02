#!/usr/bin/env python3
"""Physical desktop visualization; collision padding remains in PlanningScene."""
import argparse
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker


def physical_desk_marker(installation):
    desk=installation['desk']
    matrix=np.asarray(installation['desk_to_base']['matrix'],float)
    center=matrix@np.array([desk['size_x_m']/2,desk['size_y_m']/2,
                            desk['top_z_m']-desk['thickness_m']/2,1.])
    marker=Marker(type=Marker.CUBE,ns='physical_desk',id=0)
    marker.header.frame_id='base_link'
    marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,center[:3])
    q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
    marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w=map(float,q)
    marker.scale.x=float(desk['size_x_m']);marker.scale.y=float(desk['size_y_m']);marker.scale.z=float(desk['thickness_m'])
    marker.color.r=marker.color.g=marker.color.b=.55;marker.color.a=1.
    return marker


def parse_args(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--installation',required=True,type=Path)
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main():
    args=parse_args()
    # Imports via installed share directory for the console entry point.
    import sys
    from ament_index_python.packages import get_package_share_directory
    sys.path.insert(0,str(Path(get_package_share_directory('d1t_moveit_config'))/'launch'))
    from grasp01_core import guard_environment
    guard_environment()
    marker=physical_desk_marker(json.loads(args.installation.read_text()))
    rclpy.init();node=rclpy.create_node('grasp01_desk_visual')
    publisher=node.create_publisher(Marker,'/grasp01/physical_desk',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(marker)
    node.create_timer(1.,lambda:publisher.publish(marker))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
