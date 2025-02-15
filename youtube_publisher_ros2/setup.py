from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'youtube_publisher_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Marker file so ament can find your package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install your package.xml
        ('share/' + package_name, ['package.xml']),
        # Install your launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Panagiotis Filntisis',
    author_email='pfilntisis@athenarc.gr',
    maintainer='Panagiotis Filntisis',
    maintainer_email='pfilntisis@athenarc.gr',
    description='ROS2 package that publishes video frames from a YouTube link.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'youtube_publisher_node = youtube_publisher_ros2.youtube_publisher_node:main'
        ],
    },
)
