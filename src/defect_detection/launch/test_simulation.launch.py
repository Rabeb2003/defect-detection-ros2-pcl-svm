from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, Timer
import os


def generate_launch_description():
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            os.path.dirname(__file__), 
            '..', 
            'config', 
            'defect_detection.rviz'
        )]
    )
    
    # Point cloud publisher (simulated data)
    cloud_publisher = Node(
        package='defect_detection',
        executable='cloud_publisher',
        name='cloud_publisher',
        output='screen'
    )
    
    # Defect detection node
    defect_detection_node = Node(
        package='defect_detection',
        executable='defect_detector',
        name='defect_detection_node',
        output='screen',
        parameters=[{
            'cad_model_path': '',
            'use_icp': True,
            'enable_classification': True,
            'publish_markers': True,
        }]
    )
    
    return LaunchDescription([
        rviz_node,
        cloud_publisher,
        defect_detection_node,
    ])
