#
# Launch JetBot for CPU-based platforms (Rubik Pi / Raspberry Pi).
# Uses v4l2_camera for unaccelerated camera capture.
#
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    motor_controller_arg = DeclareLaunchArgument(
        'motor_controller',
        default_value='motors_waveshare',
        description='Motor controller executable (motors_waveshare or motors_sparkfun)'
    )

    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 camera device'
    )

    width_arg = DeclareLaunchArgument('width', default_value='320')
    height_arg = DeclareLaunchArgument('height', default_value='240')
    fps_arg = DeclareLaunchArgument('fps', default_value='15.0')

    motor_controller = Node(
        package='jetbot_ros',
        executable=LaunchConfiguration('motor_controller'),
        output='screen',
        emulate_tty=True
    )

    video_source = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[
            {'video_device': LaunchConfiguration('video_device')},
            {'image_width': LaunchConfiguration('width')},
            {'image_height': LaunchConfiguration('height')},
            {'fps': LaunchConfiguration('fps')},
        ],
        remappings=[
            ('/image_raw', '/jetbot/camera/image_raw'),
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        motor_controller_arg,
        video_device_arg,
        width_arg,
        height_arg,
        fps_arg,
        motor_controller,
        video_source,
    ])
