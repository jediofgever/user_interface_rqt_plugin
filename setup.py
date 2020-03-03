#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['user_interface_rqt_plugin'],
    package_dir={'': 'src'}
)

setup(**d)
