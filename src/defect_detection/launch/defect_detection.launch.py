from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    cad_model_arg = DeclareLaunchArgument(
        'cad_model_path',
        default_value='',
        description='Path to CAD model PCD file'
    )
    
    use_icp_arg = DeclareLaunchArgument(
        'use_icp',
        default_value='true',
        description='Use ICP for registration (true) or NDT (false)'
    )
    
    enable_classification_arg = DeclareLaunchArgument(
        'enable_classification',
        default_value='true',
        description='Enable defect classification'
    )
    
    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Publish visualization markers'
    )
    
    # Get package directory
    pkg_dir = get_package_share_directory('defect_detection')
    config_file = os.path.join(pkg_dir, 'config', 'defect_detection_params.yaml')
    
    # Defect detection node
    defect_detection_node = Node(
        package='defect_detection',
        executable='defect_detector',
        name='defect_detection_node',
        output='screen',
        parameters=[
            config_file,
            {
                'cad_model_path': LaunchConfiguration('cad_model_path'),
                'use_icp': LaunchConfiguration('use_icp'),
                'enable_classification': LaunchConfiguration('enable_classification'),
                'publish_markers': LaunchConfiguration('publish_markers'),
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
    
    return LaunchDescription([
        cad_model_arg,
        use_icp_arg,
        enable_classification_arg,
        publish_markers_arg,
        defect_detection_node,
    ])
