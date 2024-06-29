import os
import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory

def generate_launch_description():

    package_name = DeclareLaunchArgument(
        "package_name",
        default_value=TextSubstitution(text="opcua_client_core"),
        description="Name of the package.",
    )

    driver_name = DeclareLaunchArgument(
        "driver_name",
        default_value=TextSubstitution(text="ros2_opcua::LifecycleOpcUAClient"),
        description="Name of the driver.",
    )

    endpoint_url = DeclareLaunchArgument(
        "endpoint_url",
        default_value=TextSubstitution(text="opc.tcp://localhost:4840"),
        description="URL of the OPC UA server.",
    )

    dt_config_arg = DeclareLaunchArgument(
        "dt_config",
        default_value=TextSubstitution(text=os.path.join(get_package_share_directory("opcua_client_core"), "config", "datatypes.yaml")),
        description="Path to the datatypes configuration file.",
    )

    ld = launch.LaunchDescription()
    logging = launch.actions.GroupAction(
        actions=[
            launch.actions.LogInfo(msg=LaunchConfiguration("dt_config")),
            launch.actions.LogInfo(msg=LaunchConfiguration("package_name")),
            launch.actions.LogInfo(msg=LaunchConfiguration("driver_name")),
            launch.actions.LogInfo(msg=LaunchConfiguration("endpoint_url")),
        ]
    )
    lifecycle_device_container_node = launch_ros.actions.LifecycleNode(
        name="device_container_node",
        namespace="",
        package="opcua_client_core",
        output="screen",
        executable="device_container_node",
        parameters=[
            {"dt_config": LaunchConfiguration("dt_config")},
            {"package": LaunchConfiguration("package_name")},
            {"driver": LaunchConfiguration("driver_name")},
            {"endpoint_url": LaunchConfiguration("endpoint_url")},
        ],
    )

    ld.add_action(package_name)
    ld.add_action(driver_name)
    ld.add_action(endpoint_url)
    ld.add_action(dt_config_arg)
    ld.add_action(logging)
    ld.add_action(lifecycle_device_container_node)

    return ld
