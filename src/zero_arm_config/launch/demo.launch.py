import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. MoveItConfigsBuilder로 기본 설정 생성
    moveit_config = MoveItConfigsBuilder("zero_arm", package_name="zero_arm_config").to_moveit_configs()

    # 2. generate_demo_launch를 호출하여 기본 LaunchDescription 객체를 먼저 받음
    demo_launch = generate_demo_launch(moveit_config)

    # 3. Servo YAML 파일을 직접 열어서 Python 딕셔너리로 로드
    servo_yaml_path = os.path.join(
        get_package_share_directory("zero_arm_config"), "config", "zero_servo.yaml"
    )
    with open(servo_yaml_path, "r") as f:
        servo_params_yaml = yaml.safe_load(f)

    # 4. 로드된 딕셔너리에서 필요한 부분을 추출하여 Servo가 인식할 수 있는 형태로 재구성
    #    {'moveit_servo': {'ros__parameters': {...}}} -> {'moveit_servo': {...}}
    servo_params = {"moveit_servo": servo_params_yaml["moveit_servo"]["ros__parameters"]}

    # 5. Servo 노드 정의
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,  # 파일 경로 대신, 위에서 만든 딕셔너리를 직접 전달
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )
    teleop_node = Node(
    	package="zero_servo_control",
    	executable="keyboard_teleop_node",
    	name='keyboard_teleop',
        output='screen',                    # 노드의 출력을 터미널에 표시
        prefix='xterm -e'                   # 별도의 터미널 창에서 실행하여 키 입력 받기
    )
    # 6. 기존 demo_launch에 servo_node를 추가
    demo_launch.add_action(servo_node)
    demo_launch.add_action(teleop_node)

    # 7. 모든 노드가 포함된 최종 LaunchDescription을 반환
    return demo_launch
