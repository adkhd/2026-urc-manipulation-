import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. MoveIt Config 생성 (Fake Hardware 사용 설정 포함)
    # MoveItConfigsBuilder는 기본적으로 mock_components를 사용할 수 있게 설정됩니다.
    moveit_config = (
        MoveItConfigsBuilder("arm_v3", package_name="arm_v3_config")
        .robot_description(file_path="config/arm_v3.urdf.xacro")
        .to_moveit_configs()
    )

    # 2. Servo 설정 파일 로드 (위에서 만든 servo_fake.yaml)
    servo_yaml = load_yaml("arm_v3_config", "config/servo_fake.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # 3. RViz 실행
    rviz_config_file = (
        get_package_share_directory("arm_v3_config") + "/config/moveit.rviz"
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
            moveit_config.robot_description_kinematics,
        ],
    )

    # 4. ROS2 Control Node (Fake System의 핵심)
    # 가상의 하드웨어를 띄워서 Servo의 명령을 받아주고, 관절 상태(joint_states)를 다시 뱉어줍니다.
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_v3_config"),
        "config",
        "ros2_controllers.yaml",
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # 5. Controller Spawner
    # 관절 상태를 발행하는 브로드캐스터
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
    )

    # Servo가 명령을 보낼 궤적 컨트롤러 (JointTrajectoryController)
    # 주의: ros2_controllers.yaml에 'zero_arm_controller'가 정의되어 있어야 함
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # 6. MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node", 
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )
    
    # 7. Robot State Publisher (TF 발행)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 8. 컨테이너 (입력 장치 처리용 노드들)
    container = ComposableNodeContainer(
        name="servo_input_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                parameters=[{"deadzone": 0.1, "autorepeat_rate": 20.0}],
            ),
            ComposableNode(
                package="zero_servo_control",
                plugin="zero_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            servo_node,
            container,
        ]
    )