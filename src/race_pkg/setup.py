import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'race_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 등록
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 설정 파일 등록
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='F1Tenth Racing Logic Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 여기서 파이썬 파일과 ROS 실행 명령어를 연결합니다.
            'static_obstacle_detector = race_pkg.perception.static_obstacle_detector:main',
            'scan_rate_adapter = race_pkg.perception.scan_rate_adapter:main',
            'centerline_publisher = race_pkg.planning.centerline_publisher:main',
            'fgm_node = race_pkg.planning.fgm_node:main',
            'local_planner = race_pkg.planning.local_planner:main',
        ],
    },
)
