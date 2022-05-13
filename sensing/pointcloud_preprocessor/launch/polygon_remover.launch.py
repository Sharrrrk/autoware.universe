import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ns = "pointcloud_preprocessor"
    pkg = "pointcloud_preprocessor"

    # declare launch arguments
    input_points_raw_list_param = DeclareLaunchArgument(
        "input_points_raw_list", default_value="/sensing/lidar/top/rectified/pointcloud"
    )

    output_points_raw_param = DeclareLaunchArgument(
        "output_points_raw", default_value="/pointcloud/polgon_removed"
    )

    # set crop box filter as a component
    my_component = ComposableNode(
        package=pkg,
        plugin="pointcloud_preprocessor::PolgonRemoverComponent",
        name="polgon_remover",
        parameters=[
            {
                "polygon_vertices":[0.0,  0.0,
                                    10.0, 0.0,
                                    0.0,  10.0,
                                    10.0, 10.0],
                "will_visualize": True
            }
        ],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace=ns,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[my_component],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            input_points_raw_list_param,
            output_points_raw_param,
            container,
        ]
    )
