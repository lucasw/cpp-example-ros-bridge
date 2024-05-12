import rospy
import rerun as rr
from geometry_msgs.msg import (
    Pose,
    TransformStamped,
)
from tf2_ros import (
    Buffer,
    TransformException,
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
    except rospy.exceptions.ROSTimeMovedBackwardsException as ex:
        rospy.logwarn(ex)
        return False
