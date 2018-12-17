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
from setuptools.command.test import test as TestCommand
from setuptools.command.develop import develop as DevelopCommand
from setuptools.command.install import install as InstallCommand
from os import path
import os
import sys
from imp import load_source
import distutils.log as log


rafcon_root_path = os.path.dirname(os.path.abspath(__file__))

script_path = path.realpath(__file__)
install_helper = path.join(path.dirname(script_path), "source", "rafcon", "utils", "installation.py")
installation = load_source("installation", install_helper)


class PyTest(TestCommand):
    """Run py.test with RAFCON tests

    Copied http://doc.pytest.org/en/latest/goodpractices.html#integrating-with-setuptools-python-setup-py-test-pytest-runner
    """
    # This allows the user to add custom parameters to py.test, e.g.
    # $ python setup.py test -a "-v"
    user_options = [('pytest-args=', 'a', "Arguments to pass to pytest")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        # Add further test folder with 'or my_test_folder'
        # self.pytest_args = '-vx -s -k "core or gui or share_elements or user_input"'
        self.pytest_args = '-vx -s -k "core or gui or share_elements"'
        # self.pytest_args = '-vx -s -k "user_input"'

    def run_tests(self):
        import shlex
        import pytest  # import here, cause outside the eggs aren't loaded
        test_path = path.join(path.dirname(path.abspath(__file__)), 'tests')
        rafcon_path = path.join(path.dirname(path.abspath(__file__)), 'source')
        sys.path.insert(0, test_path)
        sys.path.insert(0, rafcon_path)
        os.environ["PYTHONPATH"] = rafcon_path + os.pathsep + test_path + os.pathsep + os.environ["PYTHONPATH"]
        log.info("\nRunning pytest with the following arguments: {}\n".format(shlex.split(self.pytest_args) + [
            'tests']))
        error_number = pytest.main(shlex.split(self.pytest_args) + ['tests'])
        sys.exit(error_number)


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

global_requirements = ['pylint>=1.6,<2', 'pyyaml~=3.10', 'psutil', 'jsonconversion~=0.2.9', 'yaml_configuration~=0.1',
                       'python-gtkmvc3-dlr~=1.0.0', 'gaphas~=1.0.0rc1', 'future>=0.16,<0.18.0']

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

    setup_requires=['Sphinx>=1.4', 'libsass >= 0.15.0'] + global_requirements,
    tests_require=['pytest', 'pytest-catchlog', 'graphviz', 'pymouse'] + global_requirements,
    install_requires=global_requirements,

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
        'test': PyTest
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
