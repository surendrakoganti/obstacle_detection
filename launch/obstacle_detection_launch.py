from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
    	DeclareLaunchArgument(name='eps', default_value='0.2', description=''),
    	DeclareLaunchArgument(name='min_samples', default_value='1', description=''),
        Node(
            # Add the image publisher node here
            package='obstacle_detection', 
            executable='obstacle_detection',
            name='obstacle_detection',
            output='screen',
            parameters=[{
                'eps' : LaunchConfiguration('eps'),
                'min_samples' : LaunchConfiguration('min_samples')
                }]
        )
    ])
    
    
    
    
    
    
