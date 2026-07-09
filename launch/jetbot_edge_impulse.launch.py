#
# Launch JetBot motor control plus the Edge Impulse detector.
#
# The edgeimpulse_ros detector no longer opens a camera itself; it subscribes to
# a sensor_msgs/Image topic. This launch starts a v4l2_camera driver that
# publishes to ``image_topic`` and points the detector at that topic.
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    motor_controller_arg = DeclareLaunchArgument(
        "motor_controller",
        default_value="motors_sparkfun",
        description="Motor controller executable from jetbot_ros",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        description="Absolute path to the Edge Impulse .eim file",
    )
    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/video0",
        description="V4L2 device for the USB camera driver",
    )
    width_arg = DeclareLaunchArgument("width", default_value="320")
    height_arg = DeclareLaunchArgument("height", default_value="240")
    fps_arg = DeclareLaunchArgument("fps", default_value="15.0")
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/jetbot/camera/image_raw",
        description="Image topic published by the camera driver and consumed by the detector",
    )
    image_qos_arg = DeclareLaunchArgument(
        "image_qos",
        default_value="sensor_data",
        description="sensor_data | reliable | default (must match the camera driver)",
    )
    confidence_threshold_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="-1.0",
        description="Minimum detection score to publish; <0 uses the model default",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="jetbot_camera",
        description="Override the source image frame_id on published detections",
    )
    publish_debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image",
        default_value="false",
        description="Publish an annotated debug image",
    )

    motor_controller = Node(
        package="jetbot_ros",
        executable=LaunchConfiguration("motor_controller"),
        output="screen",
        emulate_tty=True,
    )

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[
            {"video_device": LaunchConfiguration("video_device")},
            {"image_width": LaunchConfiguration("width")},
            {"image_height": LaunchConfiguration("height")},
            {"fps": LaunchConfiguration("fps")},
        ],
        remappings=[("/image_raw", LaunchConfiguration("image_topic"))],
        output="screen",
        emulate_tty=True,
    )

    detector = Node(
        package="edgeimpulse_ros",
        executable="edgeimpulse_detector",
        name="edgeimpulse_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "model_path": LaunchConfiguration("model_path"),
                "image_topic": LaunchConfiguration("image_topic"),
                "image_transport": "raw",
                "image_qos": LaunchConfiguration("image_qos"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "frame_id_override": LaunchConfiguration("frame_id"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            }
        ],
    )

    return LaunchDescription(
        [
            motor_controller_arg,
            model_path_arg,
            video_device_arg,
            width_arg,
            height_arg,
            fps_arg,
            image_topic_arg,
            image_qos_arg,
            confidence_threshold_arg,
            frame_id_arg,
            publish_debug_image_arg,
            motor_controller,
            camera,
            detector,
        ]
    )
