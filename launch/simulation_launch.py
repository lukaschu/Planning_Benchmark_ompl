import launch
import launch_ros.actions
import os
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_path = FindPackageShare(package='benchmark_planning').find('benchmark_planning') 
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='benchmark_planning',
            executable='simulation',
            parameters=[os.path.join(pkg_path, 'config/simulation_params.yaml')] 
        )
    ])