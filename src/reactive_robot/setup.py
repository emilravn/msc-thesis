from setuptools import setup
import os
from glob import glob

package_name = 'reactive_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfr',
    maintainer_email='sfr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_publisher = reactive_robot.encoder_publisher:main',
            'ultrasonic_publisher = reactive_robot.ultrasonic_publisher:main',
            'motor_subscriber = reactive_robot.motor_subscriber:main',
            'crop_follower_node = reactive_robot.crop_follower_node:main',
        ],
    },
)
