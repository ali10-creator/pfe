from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'teleop_manete'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+ '/launch', glob('launch/*.launch.py'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyes',
    maintainer_email='lyes@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['lyes = teleop_manete.manete_control:main',
                            'joy_cmd_vel = teleop_manete.joy_cmd_vel:main',
        ],
    },
)
