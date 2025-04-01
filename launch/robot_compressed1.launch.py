from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('brightness', default_value='-10'),
        DeclareLaunchArgument('multiplier_bottom', default_value='1.0'),
        DeclareLaunchArgument('multiplier_top', default_value='0.45'),
        DeclareLaunchArgument('divisor', default_value='5.0'),
        DeclareLaunchArgument('saturation', default_value='10'),
        DeclareLaunchArgument('cam_align', default_value='0'),
        DeclareLaunchArgument('islane', default_value='True'),
        
        Node(
            package='lane_following_cam',
            executable='lane_detect',
            output='screen',
            parameters=[{
                'raw_image': False, # True for raw image, False for compressed image
                'image_topic': 'image_raw/compressed',
                'brightness': LaunchConfiguration('brightness'),
                'multiplier_bottom': LaunchConfiguration('multiplier_bottom'),
                'multiplier_top': LaunchConfiguration('multiplier_top'),
                'divisor': LaunchConfiguration('divisor'),
                'saturation': LaunchConfiguration('saturation'),
                'cam_align': LaunchConfiguration('cam_align'),
                'islane': LaunchConfiguration('islane')
            }],
        ),
    ])