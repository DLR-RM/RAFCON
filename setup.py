#!/usr/bin/env python2.7

from setuptools import setup, find_packages
from setuptools.command.test import test as TestCommand
from os import path
import os
import sys


def read_version_from_pt_file():
    pt_file_name = 'rafcon.pt'
    pt_file_path = path.join(path.dirname(path.realpath(__file__)), pt_file_name)
    with open(pt_file_path) as pt_file:
        for line in pt_file:
            if line.strip().startswith('VERSION'):
                parts = line.split('=')
                version = parts[1].strip()
                return version
    return 0


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


global_requirements = ['astroid', 'pylint', 'pyyaml', 'psutil']

setup(
    name='RAFCON',
    version=read_version_from_pt_file(),
    url='https://rmintra01.robotic.dlr.de/wiki/RAFCON',
    license='',
    author='Sebastian Brunner, Rico Belder, Franz Steinmetz',
    author_email='sebastian.brunner@dlr.de, rico.belder@dlr.de, franz.steinmetz@dlr.de',
    description='RMC awesome flow control -- This is a package to program and visualize complex robot tasks.',

    packages=find_packages('source'),  # include all packages under src
    package_dir={'': 'source'},   # tell distutils packages are under src

    package_data={
        # Include all config files
        '': ['*.yaml'],
        # Include all glade files
        'mvc': ['glade/*.glade'],
    },

    # Project uses reStructuredText, so ensure that the docutils get
    # installed or upgraded on the target machine
    # install_requires=['docutils>=0.3', 'numpy', 'OpenGL', 'twisted', 'gtkmvc', 'parser', 'gtk', 'astroid', 'json',
    #                   'pyyaml'],

    setup_requires=['Sphinx>=1.4', 'Pygments>=2.0'] + global_requirements,
    tests_require=['pytest', 'pytest-catchlog'] + global_requirements,
    install_requires=global_requirements,

    cmdclass={'test': PyTest}
)



