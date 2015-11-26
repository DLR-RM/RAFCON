from setuptools import setup, find_packages
from os import path


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
    install_requires=['docutils>=0.3']
)

