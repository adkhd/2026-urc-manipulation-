import os
from glob import glob  # [추가 1] glob 모듈 임포트
from setuptools import find_packages, setup

package_name = 'typing_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [추가 2] launch 폴더 안의 모든 .launch.py 파일을 install/share/.../launch로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='MoveIt2 Python Planning Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planning = typing_control.planning:main',
        ],
    },
)