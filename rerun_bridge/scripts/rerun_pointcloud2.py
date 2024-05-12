#!/usr/bin/env python3
"""
Lucas Walter
April 2024

Turn an PointCloud2 into a Rerun visualization

adapted from rerun/examples/python/ros_node
"""

import argparse
import sys

import rerun as rr
import rospy
import sensor_msgs.point_cloud2 as pc2
from rerun_bridge import log_tf_as_transform3d
from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import ColorRGBA
from tf2_ros import (
    Buffer,
    TransformListener,
)


def make_grid(spacing=1.0,
              num=10,
              color=[200, 200, 200, 128],
              radius=0.005,
              ) -> rr.LineStrips3D:
    x0 = -spacing * num * 0.5
    x1 = spacing * num * 0.5
    y0 = -spacing * num * 0.5
    y1 = spacing * num * 0.5
    z0 = 0.0
    grid_strips = []
    colors = []
    radii = []
    for i in range(num + 1):
        xc = x0 + i * spacing
        yc = y0 + i * spacing

        grid_strips.append([[xc, y0, z0], [xc, y1, z0]])
        colors.append(color)
        radii.append(radius)

        grid_strips.append([[x0, yc, z0], [x1, yc, z0]])
        colors.append(color)
        radii.append(radius)

    return rr.LineStrips3D(grid_strips, colors=colors, radii=radii)


class RosPointCloud2ToRerun():
    def __init__(self):
        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.parent_frame = rospy.get_param("~frame", "map")
        rospy.logwarn(self.parent_frame)

        # collect some tf transforms
        try:
            rospy.sleep(1.0)
        except rospy.exceptions.ROSTimeMovedBackwardsException as ex:
            rospy.logwarn(ex)

        rospy.loginfo("creating grid")
        # Log a bounding box as a visual placeholder for the map
        if False:
            rr.log(
                f"{self.parent_frame}/box",
                rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
                timeless=True,
            )

        rr.log(
            f"{self.parent_frame}/grid",
            make_grid(spacing=10.0, num=100),
            timeless=True,
        )

        # Subscriptions
        self.info_sub = rospy.Subscriber(
            "point_cloud",
            PointCloud2,
            self.point_cloud_callback,
            queue_size=3,
        )

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        """Log `MarkerArray` with `log_image` using `cv_bridge`."""

        # TODO(lucasw) need a function that will turn a parent-child relation ship
        # into a single string with all the intermediate frames along the way
        # - tf_demo tf_tree.py probably has a good example of that

        log_tf_as_transform3d(self.tf_buffer, self.parent_frame,
                              msg.header.frame_id, msg.header.stamp)

        # field_names = [field.name for field in msg.fields]
        field_names = ["x", "y", "z"]
        pts = pc2.read_points(msg, skip_nans=True, field_names=field_names)
        # print(cloud_data[0])

        rr_points = rr.Points3D(list(pts))  # , colors=colors)
        # TODO(lucasw) use topic instead of frames?
        rr.log(f"{self.parent_frame}/{msg.header.frame_id}/point_cloud", rr_points)


def main():
    parser = argparse.ArgumentParser(description="Republish a PointCloud2 from ROS to Rerun.")
    rr.script_add_args(parser)
    args, unknown_args = parser.parse_known_args()

    # Any remaining args go to rospy
    rospy_args = [sys.argv[0]]
    rospy_args.extend(unknown_args)
    rospy.init_node("ros_point_cloud_to_rerun", argv=rospy_args)

    recording_id = rospy.get_param("~recording_id", "ros_to_rerun")

    rr.script_setup(args, "ros_point_cloud_to_rerun", recording_id=recording_id)

    _ = RosPointCloud2ToRerun()

    rospy.spin()


if __name__ == "__main__":
    main()
