#!/usr/bin/env python2.7

# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from __future__ import print_function
from setuptools import setup, find_packages
from setuptools.command.develop import develop as DevelopCommand
from setuptools.command.install import install as InstallCommand
from os import path
import os
from imp import load_source


rafcon_root_path = os.path.dirname(os.path.abspath(__file__))

script_path = path.realpath(__file__)
install_helper = path.join(path.dirname(script_path), "source", "rafcon", "utils", "installation.py")
installation = load_source("installation", install_helper)


class PostDevelopCommand(DevelopCommand):
    """Post installation step for development mode
    """
    def run(self):
        installation.install_fonts()
        installation.install_gtk_source_view_styles()
        installation.install_libraries()


class PostInstallCommand(InstallCommand):
    """Post installation step for installation mode
    """
    def run(self):
        InstallCommand.run(self)
        installation.install_fonts()
        installation.install_gtk_source_view_styles()
        installation.install_libraries()


# read version from VERSION file
# this might throw Exceptions, which are purposefully not caught as the version is a prerequisite for installing rafcon
version_file_path = os.path.join(rafcon_root_path, "VERSION")
with open(version_file_path, "r") as f:
    content = f.read().splitlines()
    version = content[0]

readme_file_path = os.path.join(rafcon_root_path, "README.rst")
with open(readme_file_path, "r") as f:
    long_description = f.read()

global_requirements = ['pylint>=1.6,<2', 'psutil', 'jsonconversion~=0.2.9', 'yaml_configuration~=0.1',
                       'python-gtkmvc3-dlr~=1.0.0', 'gaphas~=1.0.0rc1', 'future>=0.16,<0.18.0']

test_requirements = ['pytest', 'pytest-timeout', 'pytest-catchlog', 'pytest-mock', 'graphviz', 'pyuserinput']
test_requirements += global_requirements

setup(
    name='rafcon',
    version=version,
    url='https://github.com/DLR-RM/RAFCON',
    license='EPL',
    author='Sebastian Brunner, Rico Belder, Franz Steinmetz',
    author_email='sebastian.brunner@dlr.de, rico.belder@dlr.de, franz.steinmetz@dlr.de',
    description='Develop your robotic tasks with hierarchical state machines using an intuitive graphical user '
                'interface',
    long_description=long_description,

    packages=find_packages("source"),
    package_dir={'': "source"},  # tell distutils packages are under source

    package_data={
        # Include pylint and logging config
        'rafcon': ['pylintrc', 'logging.conf'],
        # Include core and GUI config
        'rafcon.core': ['config.yaml'],
        'rafcon.gui': ['gui_config.yaml'],
        # Include all glade files
        'rafcon.gui.glade': ['*.glade']
    },

    data_files=installation.generate_data_files(),

    setup_requires=['pytest-runner', 'libsass >= 0.15.0'],
    tests_require=test_requirements,
    install_requires=global_requirements,

    extras_require={
        'testing': test_requirements
    },

    sass_manifests={
        'rafcon': {
            'sass_path': 'gui/assets/share/themes/RAFCON/sass',
            'css_path': 'gui/assets/share/themes/RAFCON/gtk-3.0',
            'strip_extension': True
        }
    },

    entry_points={
        'console_scripts': [
            'rafcon_core = rafcon.core.start:main'
        ],
        'gui_scripts': [
            'rafcon_execution_log_viewer = rafcon.gui.execution_log_viewer:main',
            'rafcon = rafcon.gui.start:main'
        ]
    },

    cmdclass={
        'develop': PostDevelopCommand,
        'install': PostInstallCommand,
    },

    keywords=('state machine', 'robotic', 'FSM', 'development', 'GUI', 'visual programming'),
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Environment :: X11 Applications :: GTK',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Manufacturing',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Eclipse Public License 1.0 (EPL-1.0)',
        'Natural Language :: English',
        'Operating System :: Unix',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development',
        'Topic :: Utilities'
    ],

    zip_safe=True
)
