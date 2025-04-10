from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multi-threaded container, alternativt "component_container" om nødvendig
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',  # Plugin-navnet for rectification-noden
                name='rectify_node'
            )
        ],
        output='screen',
    )
    return LaunchDescription([container])
