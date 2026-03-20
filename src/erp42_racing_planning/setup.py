import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'erp42_racing_planning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # resource files
        (os.path.join('share', package_name, 'resource'),
         [
             'resource/waypoint_smoothed.csv',
             'resource/R1.csv',
             'resource/L1.csv',
             'resource/fix_utm_v2.csv',
             'resource/R1_sejong.csv',
             'resource/L1_sejong.csv',
         ]),

        # launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seoyunseo',
    maintainer_email='seoyunseo@todo.todo',
    description='ERP42 Racing Planning Pipeline (Behavior -> Motion Planning -> Safety -> Control)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_builder_node = erp42_racing_planning.nodes.scene_builder_node:main',
            'behavior_planner_node = erp42_racing_planning.nodes.behavior_planner_node:main',
            'local_planner_node = erp42_racing_planning.nodes.local_planner_node:main',
            'waypoint_loader = erp42_racing_planning.nodes.waypoint_loader:main',
            'waypoints_L1_node = erp42_racing_planning.nodes.waypoints_L1_node:main',
            'waypoints_R1_node = erp42_racing_planning.nodes.waypoints_R1_node:main',
        ],
    },
)
