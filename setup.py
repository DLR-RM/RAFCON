#!/usr/bin/env python2.7

from setuptools import setup, find_packages
from setuptools.command.test import test as TestCommand
from os import path
import os
import sys


class PyTest(TestCommand):
    """Run py.test with RAFCON tests

    Copied http://doc.pytest.org/en/latest/goodpractices.html#integrating-with-setuptools-python-setup-py-test-pytest-runner
    """
    # This allows the user to add custom parameters to py.test, e.g.
    # $ python setup.py test -a "-v"
    user_options = [('pytest-args=', 'a', "Arguments to pass to pytest")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        self.pytest_args = '-vxs -p no:pytest_capturelog'

    def run_tests(self):
        import shlex
        import pytest  # import here, cause outside the eggs aren't loaded
        test_path = path.join(path.dirname(path.abspath(__file__)), 'tests')
        rafcon_path = path.join(path.dirname(path.abspath(__file__)), 'source')
        sys.path.insert(0, test_path)
        sys.path.insert(0, rafcon_path)
        os.environ["PYTHONPATH"] = rafcon_path + os.pathsep + test_path + os.pathsep + os.environ["PYTHONPATH"]
        error_number = pytest.main(shlex.split(self.pytest_args) + [path.join('tests', 'network_test')])
        if not error_number:
            error_number = pytest.main(shlex.split(self.pytest_args) + [path.join('tests', 'common')])
        sys.exit(error_number)


def get_files_in_path(*path):
    path = os.path.join(*path)
    return [os.path.join(path, filename) for filename in os.listdir(path)]


global_requirements = ['astroid', 'pylint', 'pyyaml', 'psutil', 'jsonconversion>=0.2', 'yaml_configuration',
                       'python-gtkmvc==1.99.1', 'gaphas>=0.7']

tarball_url = "https://rmc-intra02.robotic.dlr.de/~stei_fn/tarballs/"
themes_folder = os.path.join('source', 'rafcon', 'gui', 'themes')

setup(
    name='rafcon',
    version='0.9.1',
    url='https://github.com/DLR-RM/RAFCON',
    license='EPL',
    author='Sebastian Brunner, Rico Belder, Franz Steinmetz',
    author_email='sebastian.brunner@dlr.de, rico.belder@dlr.de, franz.steinmetz@dlr.de',
    description='Develop your robotic tasks with hierarchical state machines using an intuitive graphical user '
                'interface',

    packages=find_packages('source'),
    package_dir={'': 'source'},  # tell distutils packages are under src

    package_data={
        # Include all config files
        'rafcon.core': ['config.yaml'],
        # Include all glade files
        'rafcon.gui': ['gui_config.yaml', 'glade/*.glade'],
    },

    data_files=[
        ('rafcon/gui/icons', get_files_in_path(themes_folder, 'icons')),
        ('rafcon/gui/splashscreens', get_files_in_path(themes_folder, 'splashscreens')),
        ('rafcon/gui/themes/dark/gtk-2.0', [os.path.join(themes_folder, 'dark', 'gtk-2.0', 'gtkrc')]),
        ('rafcon/gui/themes/dark', [os.path.join(themes_folder, 'dark', 'colors.json')]),
        ('rafcon/gui/themes/dark/gtk-sourceview', get_files_in_path(themes_folder, 'dark', 'gtksw-styles')),
        ('rafcon/gui/themes/common/fonts/DIN Next LT Pro', get_files_in_path(themes_folder, 'fonts', 'DIN Next LT Pro')),
        ('rafcon/gui/themes/common/fonts/FontAwesome', get_files_in_path(themes_folder, 'fonts', 'FontAwesome'))
    ],

    setup_requires=['Sphinx>=1.4', 'Pygments>=2.0'] + global_requirements,
    tests_require=['pytest', 'pytest-catchlog'] + global_requirements,
    install_requires=global_requirements,

    dependency_links=[
        tarball_url + "common-python_yaml_configuration-0.0.6.tar.gz#egg=yaml_configuration-0.0.6",
        tarball_url + "common-gtkmvc_dlr-e6663d8.tar.gz#egg=python-gtkmvc-1.99.1"
    ],

    entry_points={
        'console_scripts': [
            'rafcon_start = rafcon.core.start:main'
        ],
        'gui_scripts': [
            'rafcon_start_gui = rafcon.gui.start:main'
        ]
    },

    cmdclass={'test': PyTest},

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

    zip_safe=False
)
