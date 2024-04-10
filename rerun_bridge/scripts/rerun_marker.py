#!/usr/bin/env python3
"""
Lucas Walter
April 2024

Turn an rviz MarkerArray into a Rerun visualization

adapted from rerun/examples/python/ros_node
"""

import argparse

import rospy
import rerun as rr
from geometry_msgs.msg import TransformStamped
from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)
from visualization_msgs.msg import (
    MarkerArray,
)


def ros_to_rr_transform(transform: TransformStamped) -> rr.Transform3D:
    t = transform.transform.translation
    translation = [t.x, t.y, t.z]
    q = transform.transform.rotation
    rotation = rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])
    return rr.Transform3D(translation=translation, rotation=rotation)


def log_tf_as_transform3d(tf_buffer: Buffer,
                          parent_frame: str, child_frame: str, stamp: rospy.Time,
                          timeout=0.1) -> bool:
    """
    Helper to look up a transform with tf and log using `log_transform3d`.

    Note: we do the lookup on the client side instead of re-logging the raw transforms until
    Rerun has support for Derived Transforms [#1533](https://github.com/rerun-io/rerun/issues/1533)
    """

    # Do the TF lookup to get transform from child (source) -> parent (target)
    try:
        tr = tf_buffer.lookup_transform(parent_frame, child_frame,
                                        stamp, timeout=rospy.Duration(timeout))
        rr_transform = ros_to_rr_transform(tr)
        rr.log(child_frame, rr_transform)
        return True
    except TransformException as ex:
        rospy.logwarn_throttle(8.0, f"Failed to get transform: {ex}")
        return False


def make_grid(spacing=1.0,
              num=10,
              color=[200, 200, 200],
              radius=0.025,
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

    return rr.LineStrips3D(grid_strips, colors=colors)


class RosMarkerToRerun():
    def __init__(self):
        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.parent_frame = rospy.get_param("~frame", "odom")

        rospy.loginfo("creating grid")
        # Log a bounding box as a visual placeholder for the map
        if False:
            rr.log(
                "map/box",
                rr.Boxes3D(half_sizes=[3, 3, 1], centers=[0, 0, 1], colors=[255, 255, 255, 255]),
                timeless=True,
            )

        rr.log(
            "map/grid",
            make_grid(),
            timeless=True,
        )

        # Subscriptions
        self.info_sub = rospy.Subscriber(
            "marker_array",
            MarkerArray,
            self.marker_array_callback,
            queue_size=3,
        )

    def marker_array_callback(self, marker_array: MarkerArray) -> None:
        """Log `MarkerArray` with `log_image` using `cv_bridge`."""

        # TODO(lucasw) need a function that will turn a parent-child relation ship
        # into a single string with all the intermediate frames along the way
        # - tf_demo tf_tree.py probably has a good example of that

        for marker in marker_array.markers:
            # TODO(lucasw) need a lock here if multiple callbacks, otherwise time
            # will be confused?
            rr.set_time_seconds("ros_time", marker.header.stamp.to_sec())
            # rr.log(marker.header.frame_id, rr_tbd)
            self.log_tf_as_transform3d(self.tf_buffer, self.parent_frame,
                                       marker.header.frame_id, marker.header.stamp)


def main():
    parser = argparse.ArgumentParser(description="Simple example of a ROS node that republishes to Rerun.")
    rr.script_add_args(parser)
    args, unknownargs = parser.parse_known_args()
    rr.script_setup(args, "ros_marker_to_rerun")

    # Any remaining args go to rclpy
    rospy.init_node("ros_marker_to_rerun")

    _ = RosMarkerToRerun()

    rospy.spin()


if __name__ == "__main__":
    main()
