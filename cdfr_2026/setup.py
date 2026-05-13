import os
from glob import glob
from setuptools import setup

package_name = 'cdfr_2026'

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
    description='HARP2 match logic for Coupe de France de Robotique 2026.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'homologation = cdfr_2026.homologation:main',
            'gpio_reader = cdfr_2026.gpio_reader:main',
        ],
    },
)
