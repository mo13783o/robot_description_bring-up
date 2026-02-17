from setuptools import find_packages
from setuptools import setup

setup(
    name='my_robot_bringup',
    version='0.0.1',
    packages=find_packages(
        include=('my_robot_bringup', 'my_robot_bringup.*')),
)
