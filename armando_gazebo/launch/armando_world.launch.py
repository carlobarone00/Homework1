from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
armando_description_dir = get_package_share_directory('armando_description')
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    path_armando = os.path.join(
        get_package_share_directory('armando_description'))
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='armando_description')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the rfobot description relative to the package root',
                                      default_value='urdf/armando.urdf.xacro')
    
    choice_controll_arg = DeclareLaunchArgument(
    name='choice_controll',
    default_value='0',
    description='Choose which controller to use (0=position, 1=trajectory)'
    )

    choice_controll = LaunchConfiguration('choice_controll')

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    xacro_armando = os.path.join(path_armando, "urdf", "armando.urdf.xacro")

    robot_description_armando_xacro = {"robot_description": Command(['xacro ', xacro_armando, ' joint_a3_pos:=2.0', ' joint_a4_pos:=0.2'])}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_armando_xacro,
                    {"use_sim_time": True},
            ],
    )

    #declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
    #                          description='Arguments for gz_sim'))

    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '0.5', '-unpause'],
        output='screen',
    )
 
    ign = [gazebo_ignition, urdf_spawner_node]

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["position_controller", "--controller-manager", "/controller_manager"],
    condition=IfCondition(PythonExpression([choice_controll, ' == 0'])),
    )
    
    joint_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    condition=IfCondition(PythonExpression([choice_controll, ' == 1'])),
    )

    arm_controller_node_spawner = Node(
    package="armando_controller",
    executable="arm_controller_node",
    name="arm_controller_node",
    arguments=[choice_controll],
    )

    #Launch the ros2 controllers after the model spawns in Gazebo 
    delay_joint_pos_controller = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawner_node,
                on_exit=[
                    position_controller_spawner,
                    joint_trajectory_controller_spawner,
                    arm_controller_node_spawner,
                    ],
            )
        )
    )
    delay_joint_state_broadcaster = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawner_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )

    bridge_camera = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=[
        '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
        '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    ],
    output='screen'
    )

    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        choice_controll_arg,
        empty_world_launch,
        description_launch_py,
        urdf_spawner_node,
        robot_state_publisher_node,
        delay_joint_pos_controller,
        delay_joint_state_broadcaster,
        bridge_camera,
    ])
