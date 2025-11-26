from setuptools import find_packages
from setuptools import setup

setup(
    name='tutorial_inferfaces',
    version='0.0.0',
    packages=find_packages(
        include=('tutorial_inferfaces', 'tutorial_inferfaces.*')),
)
