#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory  # Importar correctamente


def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    lidar_model = LaunchConfiguration("lidar_model")
    declare_lidar_model_arg = DeclareLaunchArgument(
        "lidar_model",
        default_value="slamtec_rplidar_s1",
        description="Lidar model added to the URDF",
    )

    camera_model = LaunchConfiguration("camera_model")
    declare_camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="None",
        description="Camera model added to the URDF",
    )

    include_camera_mount = LaunchConfiguration("include_camera_mount")
    declare_include_camera_mount_arg = DeclareLaunchArgument(
        "include_camera_mount",
        default_value="False",
        description="Whether to include camera mount to the robot URDF",
    )

    # Actualizar la ruta del mundo
    world_file = PathJoinSubstitution(['/home/2024-tfg-eva-fernandez/pruebas/nao_controller/worlds', 'nao_world.sdf'])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=world_file, description="SDF world file"
    )
    
    gz_bridge = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/HeadYaw/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": world_cfg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "nao_robot",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            "-1.0",
            "-y",
            "1.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_mecanum_arg,
            declare_lidar_model_arg,
            declare_camera_model_arg,
            declare_include_camera_mount_arg,
            declare_world_arg,
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            gz_spawn_entity,
        ]
    )