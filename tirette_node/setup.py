from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tirette_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # Ajout launch/
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harp_2',
    maintainer_email='harp_2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tirette_node = tirette_node.tirette_node:main',
            'serie1 = tirette_node.serie1:main',
        ],
    },
)
