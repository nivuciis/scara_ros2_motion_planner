import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'scara_robot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
        (os.path.join('share', package_name, 'description', 'urdf'), glob('description/urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinicius RM Carvalho',
    maintainer_email='vrmc@ic.ufal.br',
    description='Implementation of motion planning algorithms (Potencial fiels and RRT) for an SCARA manipulator with ROS2/Gazebo.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinematics_node = scara_robot_simulation.scripts.kinematics_node:main',
            'jacobian_kinematics_node = scara_robot_simulation.scripts.jacobian_kinematics_node:main',
            'potential_field = scara_robot_simulation.scripts.potential_fields_planning_node:main',
            'rrtstar = scara_robot_simulation.scripts.RRTstar_planning_node:main',
            'potential_field_adaptative = scara_robot_simulation.scripts.adaptative_APF_node:main',
        ],
    },
)
