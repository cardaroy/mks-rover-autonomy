from setuptools import setup
import os
from glob import glob

package_name = 'cube_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'ultralytics'],
    zip_safe=True,
    maintainer='yao',
    maintainer_email='yao@example.com',
    description='Cube detection and simple chase controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detector_node = cube_detection.cube_detector_node:main',
            'cube_chase_controller = cube_detection.cube_chase_controller:main',
        ],
    },
)