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

    image_raw_republisher_node = Node(
        package="image_transport",
        executable="republish",
        name="image_raw_republisher",
        arguments=["raw"],
        remappings={
            "in": "/camera/image_raw",
            "out": "/camera/image_raw/compressed",
        }.items(),
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
            image_raw_republisher_node,
            tactile_notifier_node,
        ]
    )
