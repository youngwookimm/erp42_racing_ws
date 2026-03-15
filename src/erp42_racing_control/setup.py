from glob import glob

from setuptools import find_packages, setup

package_name = 'erp42_racing_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            'pure_pursuit_node = erp42_racing_control.nodes.pure_pursuit_node:main',
            'pure_pursuit_gate_node = erp42_racing_control.nodes.pure_pursuit_gate_node:main',
            'aeb_node = erp42_racing_control.nodes.aeb_node:main',
            'vehicle_cmd_gate_node = erp42_racing_control.nodes.vehicle_cmd_gate_node:main',
        ],
    },
)
