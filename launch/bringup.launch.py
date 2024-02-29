from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    v4l2_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        remappings=[("image_raw", "/camera/image_raw")],
        parameters=[{"video_device": "/dev/modot_camera"}],
    )

    face_identifier_node = Node(
        package="face_recognition_ros",
        executable="face_identifier",
        name="face_identifier",
        output="screen",
        remappings=[("image_raw", "/camera/image_raw")],
    )

    tactile_notifier_node = Node(
        package="modot_raspi",
        executable="tactile_notifier_node",
        name="tactile_notifier",
        output="screen",
    )

    return LaunchDescription(
        [
            v4l2_camera_node,
            face_identifier_node,
            tactile_notifier_node,
        ]
    )
