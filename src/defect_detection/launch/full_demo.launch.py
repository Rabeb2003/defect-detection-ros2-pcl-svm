from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    # Get package directory
    pkg_dir = get_package_share_directory('defect_detection')
    config_file = os.path.join(pkg_dir, 'config', 'defect_detection_params.yaml')
    
    # Static TF publisher node using tf2_ros
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'base_link']
    )
    
    # Cloud publisher node (simulated data)
    cloud_publisher = Node(
        package='defect_detection',
        executable='cloud_publisher',
        name='cloud_publisher',
        output='screen',
        parameters=[{
            'publish_rate': 10.0,  # 10 Hz
        }]
    )
    
    # Defect detection node
    defect_detection_node = Node(
        package='defect_detection',
        executable='defect_detector',
        name='defect_detection_node',
        output='screen',
        parameters=[
            config_file,
            {
                'cad_model_path': '',
                'use_icp': True,
                'enable_classification': True,
                'publish_markers': True,
                'icp_max_iterations': 50.0,
                'ndt_max_iterations': 35.0,
                'plane_ransac_max_iterations': 100.0,
                'min_cluster_size': 100.0,
                'max_cluster_size': 25000.0,
            }
        ],
        remappings=[
            ('/input_cloud', '/camera/depth/points'),
        ]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'defect_detection.rviz')],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    return LaunchDescription([
        enable_rviz_arg,
        # Start TF first
        static_tf_node,
        # Start publisher
        cloud_publisher,
        # Start detection node after publisher
        TimerAction(period=2.0, actions=[defect_detection_node]),
        # Start RViz after detection node
        TimerAction(period=3.0, actions=[rviz_node]),
    ])
