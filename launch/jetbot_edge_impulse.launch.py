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
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="0",
        description="V4L2 camera index used directly by edgeimpulse_ros",
    )
    score_threshold_arg = DeclareLaunchArgument(
        "score_threshold",
        default_value="0.5",
        description="Minimum detection score to publish",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="jetbot_camera",
        description="frame_id for published detections",
    )
    detections_topic_arg = DeclareLaunchArgument(
        "detections_topic",
        default_value="/edgeimpulse/detections",
        description="Detection2DArray output topic",
    )

    motor_controller = Node(
        package="jetbot_ros",
        executable=LaunchConfiguration("motor_controller"),
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
                "camera": LaunchConfiguration("camera"),
                "score_threshold": LaunchConfiguration("score_threshold"),
                "frame_id": LaunchConfiguration("frame_id"),
                "detections_topic": LaunchConfiguration("detections_topic"),
                "log_detections": False,
                "log_raw_bounding_boxes": False,
                "log_frame_summary": True,
                "publish_empty": True,
            }
        ],
    )

    return LaunchDescription(
        [
            motor_controller_arg,
            model_path_arg,
            camera_arg,
            score_threshold_arg,
            frame_id_arg,
            detections_topic_arg,
            motor_controller,
            detector,
        ]
    )
