#!/usr/bin/env python2.7

# Copyright

from setuptools import setup, find_packages
from setuptools.command.test import test as TestCommand
from setuptools.command.develop import develop as DevelopCommand
from setuptools.command.install import install as InstallCommand
from os import path
import os
import sys
from imp import load_source
import subprocess


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
        print
        print "Running pytest with the following arguments:", shlex.split(self.pytest_args) + ['tests']
        print
        error_number = pytest.main(shlex.split(self.pytest_args) + ['tests'])
        sys.exit(error_number)


def discover_fonts():
    ret = subprocess.call(['fc-cache', '-fv'])
    if ret:
        print 'Could not call command to discover new fonts: fc-cache -fv'
    else:
        print 'Called discover_fonts: fc-cache -fv'


class PostDevelopCommand(DevelopCommand):
    """Post installation step for development mode
    """
    def run(self):
        DevelopCommand.run(self)
        installation = load_source("installation", install_helper)
        installation.install_fonts()
        discover_fonts()
        installation.install_gtk_source_view_styles()
        installation.install_libraries()


class PostInstallCommand(InstallCommand):
    """Post installation step for installation mode
    """
    def run(self):
        InstallCommand.run(self)
        installation = load_source("installation", install_helper)
        installation.install_fonts()
        discover_fonts()
        installation.install_gtk_source_view_styles()
        installation.install_libraries()


def get_data_files_tuple(*path, **kwargs):
    """Return a tuple which can be used for setup.py's data_files
    
    :param tuple path: List of path elements pointing to a file or a directory of files
    :param dict kwargs: Set path_to_file to True is `path` points to a file 
    :return: tuple of install directory and list of source files
    :rtype: tuple(str, [str])
    """
    path = os.path.join(*path)
    target_path = os.path.join("share", *path.split(os.sep)[1:])  # remove source/ (package_dir)
    if "path_to_file" in kwargs and kwargs["path_to_file"]:
        source_files = [path]
        target_path = os.path.dirname(target_path)
    else:
        source_files = [os.path.join(path, filename) for filename in os.listdir(path)]
    return target_path, source_files


def get_all_files_recursivly(*path):
    """ Adds all files of the specified path to a data_files compatible list

    :param tuple path: List of path elements pointing to a directory of files
    :return: list of tuples of install directory and list of source files
    :rtype: list(tuple(str, [str]))
    """
    result_list = list()
    root_dir = os.path.join(*path)
    print "retrieving all files from foler '{}' recursivly and adding to data_files ... ".format(root_dir)

    # remove share/ (package_dir) => e.g. target_dir_sub_path will be just "libraries"
    target_dir_sub_path = os.path.join(*root_dir.split(os.sep)[1:])

    for dir_, _, files in os.walk(root_dir):
        relative_directory = os.path.relpath(dir_, root_dir)
        file_list = list()
        for fileName in files:
            relative_file = os.path.join(relative_directory, fileName)
            file_list.append(os.path.join(root_dir, relative_file))  # this is now a path relative to rafcon root folder
            # print relative_file
        if len(file_list) > 0:
            # this is valid path in ~/.local folder: e.g. share/rafcon/libraries/generic/wait
            target_path = os.path.join("share", "rafcon", target_dir_sub_path, relative_directory)
            result_list.append((target_path, file_list))
    return result_list


def generate_data_files():
    """ Generate the data_files list used in the setup function

    :return: list of tuples of install directory and list of source files
    :rtype: list(tuple(str, [str]))
    """
    assets_folder = path.join('source', 'rafcon', 'gui', 'assets')
    themes_folder = path.join(assets_folder, 'themes')
    examples_folder = path.join('share', 'examples')
    libraries_folder = path.join('share', 'libraries')

    gui_data_files = [
        get_data_files_tuple(assets_folder, 'icons'),
        get_data_files_tuple(assets_folder, 'splashscreens'),
        get_data_files_tuple(assets_folder, path.join('fonts', 'FontAwesome')),
        get_data_files_tuple(assets_folder, path.join('fonts', 'DIN Next LT Pro')),
        get_data_files_tuple(themes_folder, 'dark', 'gtk-2.0', 'gtkrc', path_to_file=True),
        get_data_files_tuple(themes_folder, 'dark', 'colors.json', path_to_file=True),
        get_data_files_tuple(themes_folder, 'dark', 'gtk-sourceview'),
    ]

    version_data_file = [("./", ["./VERSION"])]

    # print gui_data_files
    # print version_data_file

    examples_data_files = get_all_files_recursivly(examples_folder)
    # print examples_data_files
    libraries_data_files = get_all_files_recursivly(libraries_folder)
    generated_data_files = gui_data_files + examples_data_files + libraries_data_files + version_data_file
    # for elem in generated_data_files:
    #     print elem
    return generated_data_files


global_requirements = ['astroid~=1.6', 'pylint', 'pyyaml', 'psutil', 'jsonconversion~=0.2', 'yaml_configuration~=0.0',
                       'python-gtkmvc-dlr==1.99.2', 'gaphas>=0.7', 'pandas']

script_path = path.realpath(__file__)
install_helper = path.join(path.dirname(script_path), "source", "rafcon", "gui", "helpers", "installation.py")

# read version from VERSION file
# this might throw Exceptions, which are purposefully not caught as the version is a prerequisite for installing rafcon
setup_dir = os.path.dirname(__file__)
version_file_path = os.path.join(setup_dir, "VERSION")
with open(version_file_path, "r") as f:
    content = f.read().splitlines()
    version = content[0]

readme_file_path = os.path.join(setup_dir, "README.rst")
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

    packages=find_packages('source'),
    package_dir={'': 'source'},  # tell distutils packages are under src

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

    setup_requires=['Sphinx>=1.4', 'Pygments>=2.0'] + global_requirements,
    tests_require=['pytest', 'pytest-catchlog', 'graphviz', 'pymouse'] + global_requirements,
    install_requires=global_requirements,

    dependency_links=[
        "https://github.com/DLR-RM/gtkmvc3/releases/download/gtkmvc_dlr_1.99.2/python-gtkmvc-dlr-1.99.2.tar.gz"
        "#egg=python-gtkmvc-dlr-1.99.2"
    ],

    entry_points={
        'console_scripts': [
            'rafcon_start = rafcon.core.start:main',
            'rafcon_core = rafcon.core.start:main'
        ],
        'gui_scripts': [
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
        'Environment :: X11 Applications',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Manufacturing',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved',
        'Natural Language :: English',
        'Operating System :: Unix',
        'Programming Language :: Python :: 2 :: Only',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development',
        'Topic :: Utilities'
    ],

    zip_safe=True
)
