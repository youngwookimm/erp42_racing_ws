from setuptools import find_packages, setup

package_name = 'erp42_racing_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/resource',
            [
                'resource/L1.csv',
                'resource/R1.csv',
                'resource/fix_utm_v2.csv',
                'resource/fix_bag_to_planning_csv.ipynb',
            ],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/waypoints.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youngwoo',
    maintainer_email='youngwoo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoints_L1_node = erp42_racing_planning.nodes.waypoints_L1_node:main',
            'waypoints_R1_node = erp42_racing_planning.nodes.waypoints_R1_node:main',
        ],
    },
)
