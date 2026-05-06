from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'deimos_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emilio Girard',
    maintainer_email='pyloxsystems@gmail.com',
    description='DEIMOS autonomy stack: FAST-LIO2 + EKF + Nav2 + DEM + ArUco + YOLO TRT',
    license='PolyForm-Noncommercial-1.0.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dem_global_planner = deimos_autonomy.dem_global_planner:main',
            'aruco_action_server = deimos_autonomy.aruco_action_server:main',
            'yolo_trt_node = deimos_autonomy.yolo_trt_node:main',
            'goal_sequencer = deimos_autonomy.goal_sequencer:main',
        ],
    },
)
