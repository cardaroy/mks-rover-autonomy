from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'environment_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='mksneo',
    maintainer_email='mukundsrinivasan0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pointcloud_accumulator = environment_mapping.pointcloud_accumulator:main',
            'auto_velocity_control = environment_mapping.auto_velocity_control:main',
        ],
    },
)
