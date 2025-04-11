from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dsls_dea', 
            executable='dsls_dea_exe',        
            name='dsls_dea_controller',
            output='screen',
            # parameters=[{

            # }]
        )
    ])
