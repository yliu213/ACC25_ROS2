from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dsls_dea', 
            executable='dsls_dea_exe',        
            name='dsls_dea_controller',
            output='screen',
            parameters=[{
                # from ros1 rqt_reconfigure, make run time param same:
                "norm_thrust_offset_": 0.018, 
                "q_1_3_r_": 0.707106781186548, 
                "q_2_2_r_": -0.707106781186548,
            }]
        )
    ])
