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


def get_data_files_tuple(*rel_path, **kwargs):
    """Return a tuple which can be used for setup.py's data_files
    
    :param tuple path: List of path elements pointing to a file or a directory of files
    :param dict kwargs: Set path_to_file to True is `path` points to a file 
    :return: tuple of install directory and list of source files
    :rtype: tuple(str, [str])
    """
    rel_path = os.path.join(*rel_path)
    abs_path = os.path.join(rafcon_root_path, rel_path)
    target_path = os.path.join("share", *rel_path.split(os.sep)[1:])  # remove source/ (package_dir)
    if "path_to_file" in kwargs and kwargs["path_to_file"]:
        source_files = [abs_path]
        target_path = os.path.dirname(target_path)
    else:
        source_files = [os.path.join(abs_path, filename) for filename in os.listdir(abs_path)]
    return target_path, source_files


def get_data_files_recursivly(*rel_root_path, **kwargs):
    """ Adds all files of the specified path to a data_files compatible list

    :param tuple rel_root_path: List of path elements pointing to a directory of files
    :return: list of tuples of install directory and list of source files
    :rtype: list(tuple(str, [str]))
    """
    result_list = list()
    root_dir = os.path.join(rafcon_root_path, *rel_root_path)
    share_target_root = os.path.join("share", kwargs.get("share_target_root", "rafcon"))
    log.debug("recursively generating data files for folder '{}' ...".format(
        root_dir))

    for dir_, _, files in os.walk(root_dir):
        relative_directory = os.path.relpath(dir_, root_dir)
        file_list = list()
        for fileName in files:
            rel_file_path = os.path.join(relative_directory, fileName)
            abs_file_path = os.path.join(root_dir, rel_file_path)
            file_list.append(abs_file_path)
        if len(file_list) > 0:
            # this is a valid path in ~/.local folder: e.g. share/rafcon/libraries/generic/wait
            target_path = os.path.join(share_target_root, relative_directory)
            result_list.append((target_path, file_list))
    return result_list


def generate_data_files():
    """ Generate the data_files list used in the setup function

    :return: list of tuples of install directory and list of source files
    :rtype: list(tuple(str, [str]))
    """
    assets_folder = path.join('source', 'rafcon', 'gui', 'assets')
    share_folder = path.join(assets_folder, 'share')
    themes_folder = path.join(share_folder, 'themes', 'RAFCON')
    examples_folder = path.join('share', 'examples')
    libraries_folder = path.join('share', 'libraries')

    gui_data_files = [
        get_data_files_tuple(assets_folder, 'splashscreens'),
        get_data_files_tuple(assets_folder, 'fonts', 'FontAwesome'),
        get_data_files_tuple(assets_folder, 'fonts', 'DIN Next LT Pro'),
        get_data_files_tuple(themes_folder, 'gtk-3.0'),
        get_data_files_tuple(themes_folder, 'assets'),
        get_data_files_tuple(themes_folder, 'sass'),
        get_data_files_tuple(themes_folder, 'gtk-sourceview'),
        get_data_files_tuple(themes_folder, 'colors.json', path_to_file=True),
        get_data_files_tuple(themes_folder, 'colors-dark.json', path_to_file=True)
    ]
    # print("gui_data_files", gui_data_files)

    icon_data_files = get_data_files_recursivly(path.join(share_folder, 'icons'), share_target_root="icons")
    # print("icon_data_files", icon_data_files)

    locale_data_files = installation.create_mo_files()
    # example tuple
    # locale_data_files = [('share/rafcon/locale/de/LC_MESSAGES', ['source/rafcon/locale/de/LC_MESSAGES/rafcon.mo'])]
    # print("locale_data_files", locale_data_files)

    version_data_file = [("./", [os.path.join(rafcon_root_path, "VERSION")])]
    desktop_data_file = [("share/applications", [path.join(rafcon_root_path, 'share', 'applications',
                                                           'de.dlr.rm.RAFCON.desktop')])]

    examples_data_files = get_data_files_recursivly(examples_folder, share_target_root=path.join("rafcon", "examples"))
    libraries_data_files = get_data_files_recursivly(libraries_folder, share_target_root=path.join("rafcon",
                                                                                                   "libraries"))
    generated_data_files = gui_data_files + icon_data_files + locale_data_files + version_data_file + \
                           desktop_data_file + examples_data_files + libraries_data_files
    # for elem in generated_data_files:
    #     print(elem)
    return generated_data_files


global_requirements = ['pylint>=1.6,<2', 'pyyaml~=3.10', 'psutil', 'jsonconversion~=0.2.7', 'yaml_configuration~=0.1',
                       'python-gtkmvc3-dlr~=1.0.0', 'gaphas~=0.8', 'pycairo~=1.10', 'PyGObject~=3.2', 'future~=0.16']

script_path = path.realpath(__file__)
install_helper = path.join(path.dirname(script_path), "source", "rafcon", "gui", "helpers", "installation.py")
installation = load_source("installation", install_helper)

# read version from VERSION file
# this might throw Exceptions, which are purposefully not caught as the version is a prerequisite for installing rafcon
version_file_path = os.path.join(rafcon_root_path, "VERSION")
with open(version_file_path, "r") as f:
    content = f.read().splitlines()
    version = content[0]

readme_file_path = os.path.join(rafcon_root_path, "README.rst")
with open(readme_file_path, "r") as f:
    long_description = f.read()

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

    packages=find_packages(os.path.join(rafcon_root_path, "source")),
    package_dir={'': os.path.join(rafcon_root_path, "source")},  # tell distutils packages are under source

    package_data={
        # Include pylint and logging config
        'rafcon': ['pylintrc', 'logging.conf'],
        # Include core and GUI config
        'rafcon.core': ['config.yaml'],
        'rafcon.gui': ['gui_config.yaml'],
        # Include all glade files
        'rafcon.gui.glade': ['*.glade']
    },

    data_files=generate_data_files(),

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

    # dependency_links=[
    #     "https://github.com/DLR-RM/gtkmvc3/releases/download/gtkmvc_dlr_1.99.2/python-gtkmvc-dlr-1.99.2.tar.gz"
    #     "#egg=python-gtkmvc-dlr-1.99.2"
    # ],

    entry_points={
        'console_scripts': [
            'rafcon_start = rafcon.core.start:main',
            'rafcon_core = rafcon.core.start:main'
        ],
        'gui_scripts': [
            'rafcon_execution_log_viewer = rafcon.gui.execution_log_viewer:main',
            'rafcon_start_gui = rafcon.gui.start:main',
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
