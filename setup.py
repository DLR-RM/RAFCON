#!/usr/bin/env python2.7

from setuptools import setup, find_packages
from setuptools.command.test import test as TestCommand
from os import path
import os
import sys
from rafcon.utils.filesystem import read_version_from_pt_file


class PyTest(TestCommand):
    """Run py.test with RAFCON tests

    Copied from https://pytest.org/latest/goodpractises.html#integrating-with-setuptools-python-setup-py-test
    """
    # This allows the user to add custom parameters to py.test, e.g.
    # python setup.py test -a "-v"
    user_options = [('pytest-args=', 'a', "Arguments to pass to py.test")]

    def initialize_options(self):
        TestCommand.initialize_options(self)
        self.pytest_args = ['-xs']

    def finalize_options(self):
        TestCommand.finalize_options(self)
        self.test_args = []
        self.test_suite = True

    def run_tests(self):
        # import here, cause outside the eggs aren't loaded
        import pytest
        test_path = path.join(path.dirname(path.abspath(__file__)), 'source', 'test')
        rafcon_path = path.join(path.dirname(path.abspath(__file__)), 'source')
        sys.path.insert(0, test_path)
        sys.path.insert(0, rafcon_path)
        os.environ["PYTHONPATH"] =  rafcon_path + os.pathsep + test_path + os.pathsep + os.environ["PYTHONPATH"]
        error_number = pytest.main(self.pytest_args + ['source/test/all'])
        sys.exit(error_number)


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

    tests_require=['pytest'],

    cmdclass={'test': PyTest}, install_requires=['astroid', 'pylint', 'yaml']
)

