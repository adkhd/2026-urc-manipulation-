import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("zero_arm_urdf", package_name="zero_config")
        .robot_description(file_path="config/zero_arm_urdf.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("zero_config", "config/zero_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("zero_config") + "/config/moveit.rviz"
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

    # ros2_control using FakeSystem as hardware
    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("zero_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zero_arm_controller", "-c", "/controller_manager"],
    )
    
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(#조인트 tf2 발행 노드
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(#정적 tf2 브로드캐스터 노드
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "link0", "frame_id": "world"}],
            ),
            
            # ComposableNode(#작성 필요=>코드 따옴., 아마 플러그인 관련된 packagexml cmakelist 수정 필요
            #     package="zero_servo_control",
            #     plugin="zero_servo::JoyToServoPub",
            #     name="controller_to_servo_node",
            # ),
            # ComposableNode(
            #     package="joy",
            #     plugin="joy::Joy",
            #     name="joy_node",
            #     parameters=[{
            #     "deadzone": 0.12,        # <- 여기서 데드존 줌 (축 최대값의 5%)
            #     "autorepeat_rate": 200.,  # 필요 없으면 0
            #     # "dev": "/dev/input/js0",  # 디바이스도 명시하고 싶으면
            # }],
            # ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
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

    return LaunchDescription(
        [
            rviz_node,
            servo_node,
            container,
            #ros2_control_node,
            #joint_state_broadcaster_spawner,
            #panda_arm_controller_spawner,
            #teleop_node,
        ]
    )