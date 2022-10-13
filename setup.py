#!/usr/bin/env python3.6

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

from setuptools import setup, find_packages, Command
import distutils.log
import sys
import os

distutils.log.set_verbosity(distutils.log.INFO)

assert "setup.py" in os.listdir(os.curdir), "setup.py must be in current directory"


def include_as_data_file(path):
    import subprocess
    """Checks whether the given file/folder is tracked via git"""
    if ".git" not in os.listdir("."):
        # If this is not a git repo at all, regard the file as being tracked
        # This can be the case if a wheel is build from a source dist
        return True
    # Include generated CSS files
    if path.endswith(".css"):
        return True
    # Besides this, only include files tracked via git
    return subprocess.call(['git', 'ls-files', '--error-unmatch', path],
                           stderr=subprocess.STDOUT, stdout=open(os.devnull, 'w')) == 0


def get_data_files():
    data_files_dir = os.path.join(".", 'share')
    exclude_data_files_folders = ["ln", "templates"]
    data_files = []
    for directory, folders, files in os.walk(data_files_dir):
        folder_name = directory.rsplit(os.sep, 1)[1]
        if folder_name in exclude_data_files_folders:
            continue
        data_files.append((directory, [os.path.join(directory, file) for file in files
                                       if include_as_data_file(os.path.join(directory, file))]))
    return data_files


# read version from VERSION file
# this might throw Exceptions, which are purposefully not caught as the version is a prerequisite
# for installing rafcon
version_file_path = os.path.join(".", "VERSION")
with open(version_file_path, "r") as f:
    content = f.read().splitlines()
    version = content[0]

readme_file_path = os.path.join(".", "README.rst")
with open(readme_file_path, "r") as f:
    long_description = f.read()

requirements_path = os.path.join(".", "requirements.txt")
with open(requirements_path, "r") as f:
    all_requirements = f.readlines()

global_requirements = all_requirements
test_requirements = ['pytest>=6.0.0,<7.0.0', 'pytest-timeout<2', 'pytest-mock>=1.9.0,<3', 'pytest-faulthandler~=1.6.0',
                     'graphviz==0.18.2', 'pyuserinput', 'profiling==0.1.3', 'pykeyboard==0.1.2', 'pymouse==1.0',
                     'monitoring==0.9.12', 'matplotlib==2.1.1', 'objgraph==3.5.0']
setup_requirements = ['pytest-runner>=4.0.0,<7.0.0', 'libsass >= 0.15.0']

test_requirements += global_requirements


class BuildMOFiles(Command):
    description = "Create/update mo translation files"
    user_options = []

    def initialize_options(self):
        pass  # must be overridden

    def finalize_options(self):
        pass  # must be overridden

    def run(self):
        import sys
        import importlib
        sys.path.insert(0, "./source")
        i18n = importlib.import_module("rafcon.utils.i18n")
        i18n.create_mo_files(distutils.log)
        sys.path.pop()


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
        # Include pylint, logging config and localisation files
        'rafcon': ['pylintrc', 'logging.conf', 'locale/*', 'locale/*/LC_MESSAGES/*'],
        # Include core and GUI config plush splashscreens
        'rafcon.core': ['config.yaml'],
        'rafcon.gui': ['gui_config.yaml', 'design_config.yaml', 'assets/splashscreens/*'],
        # Include all glade files
        'rafcon.gui.glade': ['*.glade']
    },

    data_files=get_data_files(),

    setup_requires=setup_requirements,
    tests_require=test_requirements,
    install_requires=global_requirements,

    extras_require={
        'testing': test_requirements
    },

    cmdclass={
        'build_i18n': BuildMOFiles,
    },

    sass_manifests={
        'rafcon': {
            'sass_path': '../../share/themes/RAFCON/sass',
            'css_path': '../../share/themes/RAFCON/gtk-3.0',
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
        'Programming Language :: Python :: 3.6',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development',
        'Topic :: Utilities'
    ],
)
