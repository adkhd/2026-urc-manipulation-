from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_can_bridge',
            executable='servo_can_bridge_exe',  # setup.py console_scripts 이름
            name='servo_can_bridge',
            output='screen',
            parameters=[{
                #'mode': 'velocity',
                'mode': 'position',
                'input_topic': '/zero_arm_controller/joint_trajectory',
                'node_ids': [1,2,3,4,5],
                'joint_names': ['joint1','joint2','joint3','joint4','joint5'],
                'signs': [1,1,1,1,1],
                'gear_ratios': [285.0,33.0,33.0,49.0,49.0],
                'vel_limit_rpm': [3000.0, 1400.00, 1100.0, 2000.0, 2000.0],
                'acc_limit_rpmps': [2000.0, 500.0, 500.0, 2000.0, 2000.0],
                'dcc_limit_rpmps': [2000.0, 800.0, 800.0, 2000.0, 2000.0],
                'vel_trip_ratio': [1.3]*5,
                'pos_abs_limit_counts': [0]*5,
                'pos_rad_per_count': [1.0e-5]*5,
                'vel_radps_per_rpm': [2.0*3.1415926535/60.0]*5,
                'deadband_radps': [0.0]*5,
                'tpdo_timeout_sec': 0.5,
                'ready_wait_sec': 2.0,
                'homing_ok_forced': True,
                'publish_rate_hz': 200.0,
                'connect_on_start': True,
                'setup_on_start': True,
                'homing_joint' : [1, 1, 1, 0, 0],
                'homing_dir' : [-1, 1, 1, 1, 1],
                'homing_on' : True,
                'joint_vel_limit' : [0.5, 0.06, 0.05, 2.0, 2.0]
            }]
        )
    ])