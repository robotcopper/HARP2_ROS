import os
from glob import glob
from setuptools import setup

package_name = 'robot_teach_replay'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='floris',
    maintainer_email='floris.jousselin@neura-robotics.com',
    description='Teach & replay gamepad trajectories with collision_monitor safety overlay.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor = robot_teach_replay.safety_supervisor:main',
        ],
    },
)
