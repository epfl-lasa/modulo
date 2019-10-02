#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import os
from setuptools import setup, find_packages

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "state_representation",
    version = "0.0.1",
    author = "Baptiste Busch",
    author_email = "baptiste.busch@epfl.ch",
    description = ("A package to represent transformations, joint and robot states"),
    license = "AFL",
    url = "https://github.com/epfl-lasa/modulo/tree/master/source/lib/state_representation",
    packages=find_packages('src'),
    package_dir={'': 'src'},
    long_description=read('README.md'),
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Framework :: Robot Framework :: Library",
        "License :: OSI Approved :: Academic Free License (AFL)",
    ],
)