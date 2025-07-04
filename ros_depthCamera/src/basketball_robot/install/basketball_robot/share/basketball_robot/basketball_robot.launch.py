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
    
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode', default_value='false',
        description='Run in simulation mode without hardware'
    )

    return LaunchDescription([
        # Arguments
        serial_dev_arg,
        joy_dev_arg,
        target_distance_arg,
        camera_angle_arg,
        simulation_mode_arg,
        
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
        
        # Smart Drive Bridge Node
        Node(
            package='basketball_robot',
            executable='smart_drive_bridge',
            name='smart_drive_bridge',
            parameters=[{
                'dev': LaunchConfiguration('serial_dev'),
                'baud': 115200,
                'target_distance': LaunchConfiguration('target_distance'),
                'camera_angle': LaunchConfiguration('camera_angle'),
                'simulation_mode': LaunchConfiguration('simulation_mode'),
            }],
            output='screen'
        ),
        
        # Basketball Detector Node
        Node(
            package='basketball_robot',
            executable='basketball_detector',
            name='basketball_detector',
            parameters=[{
                'simulation_mode': LaunchConfiguration('simulation_mode'),
                'model_path': 'yolov8n.pt',
                'detection_classes': ['sports ball', 'person'],
                'detection_threshold': 0.5,
            }],
            output='screen'
        )
    ])