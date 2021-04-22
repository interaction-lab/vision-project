#!/usr/bin/env python
from setuptools import setup


setup(
    name='vision-interaction',
    packages=[
        'controllers',
        'interaction_builder',
        'interfaces',
        'vision_project_tools'
    ],
    version='0.0.0',
    install_requires=[
        'freezegun',
        'mock',
        'mongomock',
        'pydub',
        'pymongo',
        'pytest',
        'pytest-cov',
    ],
)
