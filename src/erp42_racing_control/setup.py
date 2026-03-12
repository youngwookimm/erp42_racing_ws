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
            'step_velocity_test_node = erp42_racing_control.nodes.step_velocity_test_node:main',
            'braking_distance_test_node = erp42_racing_control.nodes.braking_distance_test_node:main',
            'braking_input_test_node = erp42_racing_control.nodes.braking_input_test_node:main',
        ],
    },
)
