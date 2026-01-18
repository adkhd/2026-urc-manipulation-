from setuptools import setup

package_name = 'rmd_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # <-- 여기가 find_packages() 또는 [package_name] 이어야 함
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='RMD Control Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rmd_node = rmd_control.rmd_node:main', # <-- 실행 진입점
            'rmd_node_vel = rmd_control.rmd_node_vel:main', # <-- 실행 진입점
        ],
    },
)