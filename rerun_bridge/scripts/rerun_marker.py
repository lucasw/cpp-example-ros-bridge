#!/usr/bin/env python3
"""
Lucas Walter
April 2024

Turn an rviz MarkerArray into a Rerun visualization

adapted from rerun/examples/python/ros_node

Use with https://github.com/lucasw/visualization_tutorials/blob/obese-aggregated/visualization_marker_tutorials/scripts/example_marker_array.py
# noqa: E501
"""

import argparse
import sys

import rospy
import rerun as rr
from geometry_msgs.msg import (
    Pose,
    TransformStamped,
)
from std_msgs.msg import ColorRGBA
from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
)


def ros_pose_to_rr_transform(pose: Pose) -> rr.Transform3D:
    t = pose.position
    translation = [t.x, t.y, t.z]
    q = pose.orientation
    rotation = rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])
    return rr.Transform3D(translation=translation, rotation=rotation)


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
        rr_name = f"{parent_frame}/{child_frame}"
        rr.log(rr_name, rr_transform)
        return True
    except TransformException as ex:
        rospy.logwarn_throttle(8.0, f"Failed to get transform: {ex}")
        return False


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


def marker_color_to_rr(marker_color: ColorRGBA) -> list[int]:
    """
    convert 4 0-1.0 floats to 4 0-255 integers
    """
    return [int(marker_color.r * 255), int(marker_color.g * 255),
            int(marker_color.b * 255), int(marker_color.a * 255)]


def marker_to_rr(marker: Marker,
                 parent_frame: str,
                 radius=0.005,
                 ) -> (rr.LineStrips3D, str, rr.Transform3D):
    origin = [0.0, 0.0, 0.0]
    color = marker_color_to_rr(marker.color)
    # mpo = marker.pose.orientation
    # rotation = rr.Quaternion(xyzw=[mpo.x, mpo.y, mpo.z, mpo.w])

    rr_name = f"{parent_frame}/{marker.header.frame_id}/{marker.ns}/{marker.id}"
    rr_transform = ros_pose_to_rr_transform(marker.pose)
    rr_marker = None

    if marker.type == Marker.ARROW:
        # TODO(lucasw) the arrow isn't right
        # rot_matrix = tf.transformations.quaternion_matrix([mpo.x, mpo.y, mpo.z, mpo.w])
        vector = [marker.scale.x, 0.0, 0.0]  # (rot_matrix[0, 0:3] * marker.scale.x).tolist()
        rr_marker = rr.Arrows3D(origins=[origin], vectors=[vector], colors=[color],
                                radii=[marker.scale.y * 0.5])

    elif marker.type == Marker.CUBE:
        rr_marker = rr.Boxes3D(centers=[origin],
                               half_sizes=[[marker.scale.x * 0.5, marker.scale.y * 0.5, marker.scale.z * 0.5]],
                               # rotations=[rotation],
                               radii=radius,
                               colors=[color])

    elif marker.type == Marker.SPHERE:
        # TODO(lucasw) rviz spheres can be squashed/extended with scale xyz (each is diameter)
        # (though I'm not seeing that working right now with example_marker_array.py)
        rr_marker = rr.Points3D([origin], colors=[color], radii=[marker.scale.x * 0.5])

    elif marker.type == Marker.CYLINDER:
        rospy.logwarn_throttle(8.0, "need to construct a rerun Mesh3D cylinder")
        # TODO(lucasw) is there a light weight mesh generation python library- maybe pymesh, or pygalmesh?
        # or copy paste standalone functions here
        rr_marker = rr.Points3D([origin], colors=[color], radii=[marker.scale.x * 0.5])

    elif marker.type == Marker.LINE_STRIP:
        strips = []
        # radii = []
        for pt in marker.points:
            # radii.append(marker.scale.x)
            strips.append([pt.x, pt.y, pt.z])
        rr_marker = rr.LineStrips3D([strips], colors=[color])

    elif marker.type == Marker.LINE_LIST:
        strips = []
        colors = []
        # radii = []
        count = 0
        for pt in marker.points:
            if count % 2 == 0:
                strips.append([])
                # colors.append([])
                colors.append(color)
            # radii.append(marker.scale.x)
            strips[-1].append([pt.x, pt.y, pt.z])
            # colors[-1].append(color)
            count += 1
        rr_marker = rr.LineStrips3D(strips, colors=colors)

    elif marker.type == Marker.CUBE_LIST:
        centers = []
        half_sizes = []
        colors = []
        for pt in marker.points:
            centers.append([pt.x, pt.y, pt.z])
            half_sizes.append([marker.scale.x * 0.5, marker.scale.y * 0.5, marker.scale.z * 0.5])
            colors.append(color)

        rr_marker = rr.Boxes3D(centers=centers,
                               half_sizes=half_sizes,
                               # rotations=[rotation],
                               # TODO(lucasw) need to make a mesh for solid boxes?
                               radii=radius,
                               colors=colors)

    elif marker.type in [Marker.SPHERE_LIST, Marker.POINTS]:
        # TODO(lucasw) there isn't a rerun primitive for viewer aligned squares which is how rviz
        # renders Marker.POINTS, so using spheres for now

        centers = []
        radii = []
        colors = []
        for pt in marker.points:
            centers.append([pt.x, pt.y, pt.z])
            radii.append(marker.scale.x * 0.5)
            colors.append(color)

        rr_marker = rr.Points3D(centers, colors=colors, radii=radii)

    elif marker.type == Marker.TEXT_VIEW_FACING:
        rr_marker = rr.Points3D([origin], colors=[color], radii=[0.0], labels=[marker.text])

    elif marker.type == Marker.TRIANGLE_LIST:
        points = []
        colors = []
        for pt in marker.points:
            # TODO(lucasw) this ignores the rotation, need to create a transform for that
            points.append([pt.x, pt.y, pt.z])
            # TODO(lucasw) if the marker colors list is populated then use those
            colors.append(color)

        rr_marker = rr.Mesh3D(vertex_positions=points, vertex_colors=colors)

    else:
        rospy.logwarn(f"unsupported {marker.type}")

    return rr_marker, rr_name, rr_transform


class RosMarkerToRerun():
    def __init__(self):
        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.parent_frame = rospy.get_param("~frame", "map")
        rospy.logwarn(self.parent_frame)

        # collect some tf transforms
        rospy.sleep(1.0)

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
            log_tf_as_transform3d(self.tf_buffer, self.parent_frame,
                                  marker.header.frame_id, marker.header.stamp)
            rr_marker, rr_name, rr_transform = marker_to_rr(marker, self.parent_frame)
            rr.log(rr_name, rr_transform)
            if rr_marker is None:
                continue
            # if "curve" in marker.ns:
            #     rospy.loginfo_throttle(0.0, f"{rr_name} {marker.ns} {marker.id}")
            rr.log(rr_name, rr_marker)


def main():
    parser = argparse.ArgumentParser(description="Simple example of a ROS node that republishes to Rerun.")
    rr.script_add_args(parser)
    args, unknown_args = parser.parse_known_args()

    # Any remaining args go to rospy
    rospy_args = [sys.argv[0]]
    rospy_args.extend(unknown_args)
    rospy.init_node("ros_marker_to_rerun", argv=rospy_args)

    recording_id = rospy.get_param("~recording_id", "ros_to_rerun")

    rr.script_setup(args, "ros_marker_to_rerun", recording_id=recording_id)

    _ = RosMarkerToRerun()

    rospy.spin()


if __name__ == "__main__":
    main()
