import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "RIN",
        default_value="SRP1",
        description="Robot identification number RIN to choose between setups -- possible values: [SRP1, IQ159-1]",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):

    # load config file
    motion_config_path = os.path.join(
        get_package_share_directory('benchmark_planning'),
        'config',
        f'moveit.yaml'
    )

    with open(motion_config_path, 'r') as config_file:
        motion_config = yaml.safe_load(config_file)

    # Initialize arguments
    ros2_control_hardware_type = motion_config['ros2_control_hardware_type']
    robot_ip = motion_config['robot_ip']
    reverse_ip = motion_config['reverse_ip']
    ur_type = motion_config['ur_type']

    # Define file paths for RTDE input and output recipes
    # Will be forwared to urXX.urdf.xacro, need to be set here for the robot to work
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )

    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    # Set UR controller for motion execution
    moveit_controllers = 'joint_trajectory_controller.yaml'

    # Build MoveIt configuration for the UR robot, used for MoveIt and controlling the robot
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur10e", package_name="moveit_config_ur")
        .robot_description(
            file_path=f"urdf/{ur_type}.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": ros2_control_hardware_type,
                "robot_ip": robot_ip,
                "input_recipe_filename": input_recipe_filename,
                "output_recipe_filename": output_recipe_filename,
                "reverse_ip": reverse_ip,
            },
        )
        .joint_limits(file_path=f"config/{ur_type}/joint_limits_{motion_config['motion_core_node']['pose_config']}.yaml")
        .robot_description_semantic(file_path=f"srdf/{ur_type}.srdf.xacro")
        .trajectory_execution(file_path=f"config/{moveit_controllers}")
        .moveit_cpp(file_path="config/motion_planning.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start motion and environment nodes
    avp_planner_node = Node(
        package="benchmark_planning",
        executable="simulation",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            motion_config['motion_core_node'],
            {"use_sim_time": True},
            {"use_fake_hardware":True},
        ]
    )

    nodes_to_start = [TimerAction(
        period=1.0,
        actions=[
        avp_planner_node
        ]
    )
    ]

    return nodes_to_start