from setuptools import setup

package_name = 'servo_can_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/servo_can_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Multi-axis Servo <-> CANopen bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # ros2 run zero_servo_bridge servo_can_bridge
            'servo_can_bridge_exe = servo_can_bridge.servo_can_bridge:main',
        ],
    },
)