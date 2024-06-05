import xacro
from os.path import join
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # MoveIt! Configuration
    moveit_config = (
        MoveItConfigsBuilder("ur10e")
        .robot_description(file_path="config/ur10e_robotiq.urdf")
        .robot_description_semantic(file_path="config/ur10e_robotiq.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update({"use_sim_time": True})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )


    # Configure URDF file
    urdf_pkg_share = get_package_share_directory('ur10e_description')
    urdf_path_local = 'urdf/ur10e_robotiq_2f_85.urdf.xacro'
    urdf_path_global = join(urdf_pkg_share, urdf_path_local)
    robot_description_raw = xacro.process_file(urdf_path_global).toxml()

    # Launch Gazebo 
    gz_pkg_share = get_package_share_directory('ros_gz_sim')
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(gz_pkg_share, 'launch', 'gz_sim.launch.py')])
    )



    # Configure nodes to launch
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "ur10e_base_link"],
    )
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_raw, 
             'use_sim_time': True}
        ],
        output='both',
    )

    rviz_config_file = join(urdf_pkg_share, 'rviz', 'view_robot.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    robot_name = 'ur10e'
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', robot_name, 
            '-world', 'default'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Spawn controller nodes 

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner, robotiq_gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        move_group_node,
        rsp_node,
        rviz_node,
        launch_gazebo,
        create_node,
        delay_joint_state_broadcaster_spawner,
        delay_joint_trajectory_controller_spawner,
    ])