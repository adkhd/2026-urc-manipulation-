from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # moveit_cpp.yaml 경로 명시적으로 지정!
    moveit_config = (
        MoveItConfigsBuilder("arm_v3", package_name="arm_v3_config")
        .moveit_cpp(
            file_path=get_package_share_directory("arm_v3_config") + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    
    planning_node = Node(
        package='typing_control',
        executable='planning',
        name='moveit_py_planner',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription([planning_node])