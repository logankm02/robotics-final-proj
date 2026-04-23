from setuptools import find_packages
from setuptools import setup

setup(
    name='planning_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('planning_interfaces', 'planning_interfaces.*')),
)
