import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='seedDecompContainer',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='decomp_ros',
                    plugin='decompros::SeedDecomp',
                    name='seedDecomp'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
