#  Copyright (c) 2024 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
def setup_moveit_config():
    return (
        MoveItConfigsBuilder("franka_fr3", package_name="franka_fr3_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("franka_description"),
                "robots", "fr3", "fr3.urdf.xacro"
            ),
            mappings={
                "robot_ip": LaunchConfiguration("robot_ip"),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
                "hand": "true",  # 启用手部
            }
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("franka_fr3_moveit_config"),
                "srdf", "fr3_arm.srdf.xacro"
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory("franka_vr"),
                "config", "joint_limits.yaml"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("franka_vr"),
                "config", "kinematics.yaml"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("franka_vr"),
                "config", "fr3_controllers.yaml"
            )
        )
        .planning_pipelines(
            pipelines=["ompl"],  # List of pipelines to configure

        )
        .to_moveit_configs()
    )
def robot_description_dependent_nodes_spawner(
        context: LaunchContext,
        robot_ip,
        arm_id,
        use_fake_hardware,
        fake_sensor_commands,
        load_gripper,
        arm_prefix):

    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    arm_prefix_str = context.perform_substitution(arm_prefix)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(
        fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    franka_xacro_filepath = os.path.join(get_package_share_directory(
        'franka_description'), 'robots', arm_id_str, arm_id_str+'.urdf.xacro')
    robot_description = xacro.process_file(franka_xacro_filepath,
                                           mappings={
                                               'ros2_control': 'true',
                                               'arm_id': arm_id_str,
                                               'robot_ip': robot_ip_str,
                                               'hand': load_gripper_str,
                                               'use_fake_hardware': use_fake_hardware_str,
                                               'fake_sensor_commands': fake_sensor_commands_str,
                                               'arm_prefix': arm_prefix_str,
                                           }).toprettyxml(indent='  ')

    franka_controllers = PathJoinSubstitution(
        [FindPackageShare('franka_bringup'), 'config', 'controllers.yaml'])
    # 构建 MoveIt 配置
    moveit_config = setup_moveit_config()
    # # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("franka_vr")
        .yaml("config/fr3_real_config.yaml")  # Make sure this file exists for FR3
        .to_dict()
    }
    # Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="demo_franka_vr_vel",
        parameters=[
            servo_params,
            {"update_period": 0.01},
            {"planning_group_name": "fr3_arm"},  # Changed from panda_arm to fr3_arm
            moveit_config.to_dict(),
        ],
        output="screen",
    )
# RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[franka_controllers,
                        {'robot_description': robot_description},
                        {'arm_id': arm_id},
                        {'load_gripper': load_gripper},
                        ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),servo_node,rviz_node]


def generate_launch_description():
    arm_id_parameter_name = 'arm_id'
    arm_prefix_parameter_name = 'arm_prefix'
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    arm_id = LaunchConfiguration(arm_id_parameter_name)
    arm_prefix = LaunchConfiguration(arm_prefix_parameter_name)
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(
        fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    robot_description_dependent_nodes_spawner_opaque_function = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[
            robot_ip,
            arm_id,
            use_fake_hardware,
            fake_sensor_commands,
            load_gripper,
            arm_prefix])
 
    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='172.31.1.31',
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            arm_id_parameter_name,
            default_value='fr3',
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            arm_prefix_parameter_name,
            default_value='fr3',
            description='The prefix of the arm.'),
        
    
        robot_description_dependent_nodes_spawner_opaque_function,
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
    
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            parameters=[{'arm_id': arm_id}],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),

        Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf2_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "1", "world", "base"],
        output="screen"
        ),
        Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "-c", "/controller_manager"],
         ),
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware,
                              'arm_id':arm_id}.items(),
            condition=IfCondition(load_gripper)
        )
    ,
        # Node(package='rviz2',
        #      executable='rviz2',
        #      name='rviz2',
        #      arguments=['--display-config', rviz_file],
        #      condition=IfCondition(use_rviz)
        #      ),
        # Add FR3 Arm Controller HERE
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager'],
        #     output='screen',
        #     # Optional: Add delay to ensure previous controllers are up
        #     # prefix="bash -c 'sleep 2; $0 $@'",
        # ),
             

    ])

    return launch_description
