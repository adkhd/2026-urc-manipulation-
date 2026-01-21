from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # MoveIt 기본 설정
    moveit_config = (
        MoveItConfigsBuilder("arm_v3", package_name="arm_v3_config")
        .moveit_cpp(
            file_path=get_package_share_directory("arm_v3_config") + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    
    # 기본 파라미터
    all_params = moveit_config.to_dict()
    
    # 🔥 OMPL 파라미터를 중첩 구조로 추가
    # (moveit_cpp.yaml에 없는 경우를 위한 fallback)
    if "ompl" not in all_params or not isinstance(all_params.get("ompl"), dict):
        print("\n" + "="*60)
        print("⚠️  Adding OMPL parameters (not found in moveit_cpp.yaml)")
        print("="*60 + "\n")
        
        # 🔥 중첩된 구조로 추가!
        all_params["ompl"] = {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "path_tolerance": 0.1,
            "resample_dt": 0.1,
            "min_angle_change": 0.001,
            "default_workspace_bounds": 10.0,
            "start_state_max_bounds_error": 0.1,
            "start_state_max_dt": 0.5,
            "jiggle_fraction": 0.05,
            "max_sampling_attempts": 200,
        }
    else:
        print("\n" + "="*60)
        print("✅ OMPL Configuration from moveit_cpp.yaml")
        print("="*60)
        print(f"  OMPL params: {all_params['ompl']}")
        print("="*60 + "\n")
    
    all_params["use_sim_time"] = False
    
    planning_node = Node(
        package='typing_control',
        executable='planning',
        name='moveit_py_planner',
        output='screen',
        parameters=[all_params],
    )
    
    return LaunchDescription([planning_node])