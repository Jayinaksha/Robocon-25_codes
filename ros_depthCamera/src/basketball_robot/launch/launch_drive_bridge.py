from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    serial_dev_arg = DeclareLaunchArgument(
        'serial_dev', default_value='/dev/ttyUSB0',
        description='Serial device for Arduino communication'
    )
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev', default_value='/dev/input/js0',
        description='Joystick device'
    )
    
    target_distance_arg = DeclareLaunchArgument(
        'target_distance', default_value='2.5',
        description='Target distance for auto shooting (meters)'
    )
    
    camera_angle_arg = DeclareLaunchArgument(
        'camera_angle', default_value='55.0',
        description='Camera angle from horizontal (degrees)'
    )
    
    # Distance PID parameters
    pid_distance_p_arg = DeclareLaunchArgument(
        'pid_distance_p', default_value='0.5',
        description='Distance PID proportional gain'
    )
    
    pid_distance_i_arg = DeclareLaunchArgument(
        'pid_distance_i', default_value='0.01',
        description='Distance PID integral gain'
    )
    
    pid_distance_d_arg = DeclareLaunchArgument(
        'pid_distance_d', default_value='0.1',
        description='Distance PID derivative gain'
    )
    
    # Alignment PID parameters (for left/right movement)
    pid_alignment_p_arg = DeclareLaunchArgument(
        'pid_alignment_p', default_value='0.4',
        description='Alignment PID proportional gain'
    )
    
    pid_alignment_i_arg = DeclareLaunchArgument(
        'pid_alignment_i', default_value='0.0',
        description='Alignment PID integral gain'
    )
    
    pid_alignment_d_arg = DeclareLaunchArgument(
        'pid_alignment_d', default_value='0.05',
        description='Alignment PID derivative gain'
    )

    return LaunchDescription([
        # Arguments
        serial_dev_arg,
        joy_dev_arg,
        target_distance_arg,
        camera_angle_arg,
        pid_distance_p_arg,
        pid_distance_i_arg,
        pid_distance_d_arg,
        pid_alignment_p_arg,
        pid_alignment_i_arg,
        pid_alignment_d_arg,
        
        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Drive Serial Bridge Node
        Node(
            package='basketball_robot',
            executable='drive_serial_bridge',
            name='drive_serial_bridge',
            parameters=[{
                'dev': LaunchConfiguration('serial_dev'),
                'baud': 115200,
                'target_distance': LaunchConfiguration('target_distance'),
                'camera_angle': LaunchConfiguration('camera_angle'),
                'pid_distance_p': LaunchConfiguration('pid_distance_p'),
                'pid_distance_i': LaunchConfiguration('pid_distance_i'),
                'pid_distance_d': LaunchConfiguration('pid_distance_d'),
                'pid_alignment_p': LaunchConfiguration('pid_alignment_p'),
                'pid_alignment_i': LaunchConfiguration('pid_alignment_i'),
                'pid_alignment_d': LaunchConfiguration('pid_alignment_d'),
            }],
            output='screen'
        )
    ])