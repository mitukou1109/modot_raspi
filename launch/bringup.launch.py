from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_result_image = LaunchConfiguration("publish_result_image")

    publish_result_image_argument = DeclareLaunchArgument(
        "publish_result_image", default_value="true"
    )

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

    face_identifier_node = Node(
        package="face_recognition_ros",
        executable="face_identifier",
        name="face_identifier",
        output="screen",
        remappings=[("image_raw", "/camera/image_raw")],
        parameters=[
            {
                "publish_result_image": publish_result_image,
                "resize_height": 160,
            }
        ],
    )

    result_image_republisher_node = Node(
        package="image_transport",
        executable="republish",
        name="result_image_republisher",
        arguments=["raw"],
        remappings={
            "in": "/face_identifier/result_image",
            "out": "/face_identifier/result_image/compressed",
        }.items(),
        condition=IfCondition(publish_result_image),
    )

    tactile_notifier_node = Node(
        package="modot_raspi",
        executable="tactile_notifier_node",
        name="tactile_notifier",
        output="screen",
    )

    return LaunchDescription(
        [
            publish_result_image_argument,
            v4l2_camera_node,
            image_raw_republisher_node,
            face_identifier_node,
            result_image_republisher_node,
            tactile_notifier_node,
        ]
    )
