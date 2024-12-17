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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Para poder lanzar el mundo
    world_file = PathJoinSubstitution(['/home/2024-tfg-eva-fernandez/pruebas/moving_nao/worlds', 'nao_world.sdf'])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=world_file, description="SDF world file"
    )
    
    # Para lanzar simulación de gazebo
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
    
    # Ros bridges para controlar articulaciones de NAO
    gz_bridge_1 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/HeadPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_2 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/HeadYaw/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_3 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LAnklePitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_4 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LAnkleRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_5 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LElbowRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_6 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LElbowYaw/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_7 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LHipPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_8 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LHipRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_9 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LHipYawPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_10 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LKneePitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_11 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LShoulderPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_12 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LShoulderRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_13 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/LWristYaw/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )

    gz_bridge_14 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RAnklePitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_15 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RAnkleRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_16 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RElbowRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_17 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RElbowYaw/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_18 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RHipPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_19 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RHipRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_20 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RHipYawPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_21 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RKneePitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_22 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RShoulderPitch/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_23 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RShoulderRoll/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    gz_bridge_24 = Node(
    	package="ros_gz_bridge",
    	executable="parameter_bridge",
    	name="gz_bridge",
    	arguments=[
    	   "/RWristYaw/cmd_pos" + "@std_msgs/msg/Float64" + "@gz.msgs.Double",
    	],
    	output="screen",
    )
    
    
    gz_bridge_25 = Node(
        package='ros_gz_image',
        executable='parameter_bridge',
        arguments=['/NAO/camera/camera_info'],
        output='screen',
    )
    
    gz_bridge_25 = Node(
        package='ros_gz_image',
        executable='parameter_bridge',
        arguments=['/NAO/camera/image_raw'],
        output='screen',
    )
    
    return LaunchDescription(
        [
            declare_world_arg,
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge_1,
            gz_bridge_2,
            gz_bridge_3,
            gz_bridge_4,
            gz_bridge_5,
            gz_bridge_6,
            gz_bridge_7,
            gz_bridge_8,
            gz_bridge_9,
            gz_bridge_10,
            gz_bridge_11,
            gz_bridge_12,
            gz_bridge_13,
            gz_bridge_14,
            gz_bridge_15,
            gz_bridge_16,
            gz_bridge_17,
            gz_bridge_18,
            gz_bridge_19,
            gz_bridge_20,
            gz_bridge_21,
            gz_bridge_22,
            gz_bridge_23,
            gz_bridge_24,
            gz_bridge_25,
            gz_biridge_26,
        ]
    )
