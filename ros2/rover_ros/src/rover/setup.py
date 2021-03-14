import os

from setuptools import setup
from glob import glob

package_name = 'rover'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('description/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taylan Pince',
    maintainer_email='taylanpince@gmail.com',
    description='Rover with homemade LIDAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar = rover.lidar:main',
            'joint_states = rover.joint_states:main',
        ],
    },
)
