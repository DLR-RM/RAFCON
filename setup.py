#!/usr/bin/env python2.7

from setuptools import setup, find_packages
from setuptools.command.test import test as TestCommand
from setuptools.command.develop import develop as DevelopCommand
from setuptools.command.install import install as InstallCommand
from os import path
import os
import sys
import shutil
from distutils import log

try:
    import gtk
except ImportError:
    gtk = None
try:
    import glib
except ImportError:
    glib = None

log.set_verbosity(log.DEBUG)


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


def install_fonts():
    if not gtk:
        log.warn("No GTK found. Will not install fonts.")
        return

    tv = gtk.TextView()
    try:
        context = tv.get_pango_context()
    except Exception as e:
        log.error("Could not get pango context. Will not install fonts: {}".format(e))
        return
    if not context:  # A Pango context is not always available
        log.warn("Could not get pango context. Will not install fonts.")
        return
    existing_fonts = context.list_families()
    existing_font_names = [font.get_name() for font in existing_fonts]

    user_otf_fonts_folder = os.path.join(os.path.expanduser('~'), '.fonts')

    try:
        for font_name in ["DIN Next LT Pro", "FontAwesome"]:
            if font_name in existing_font_names:
                log.debug("Font '{0}' found".format(font_name))
                continue

            log.info("Installing font '{0}' to {1}".format(font_name, user_otf_fonts_folder))
            if not os.path.isdir(user_otf_fonts_folder):
                os.makedirs(user_otf_fonts_folder)

            # A font is a folder one or more font faces
            fonts_folder = os.path.join(assets_folder, "fonts", font_name)
            for font_face in os.listdir(fonts_folder):
                target_font_file = os.path.join(user_otf_fonts_folder, font_face)
                source_font_file = os.path.join(fonts_folder, font_face)
                shutil.copy(source_font_file, target_font_file)
    except IOError as e:
        log.error("Could not install fonts, IOError: {}".format(e))
        return


def install_gtk_source_view_styles():
    if glib:
        user_data_folder = glib.get_user_data_dir()
    else:
        user_data_folder = os.path.join(os.path.expanduser('~'), '.local', 'share')
    user_source_view_style_path = os.path.join(user_data_folder, 'gtksourceview-2.0', 'styles')

    try:
        if not os.path.exists(user_source_view_style_path):
            os.makedirs(user_source_view_style_path)

        # Copy all .xml source view style files from all themes to local user styles folder
        themes_path = os.path.join(assets_folder, "themes")
        for theme in os.listdir(themes_path):
            theme_source_view_path = os.path.join(themes_path, theme, "gtk-sourceview")
            if not os.path.isdir(theme_source_view_path):
                continue
            for style_filename in os.listdir(theme_source_view_path):
                if not style_filename.endswith(".xml"):
                    continue
                log.info("Installing GTKSourceView style '{}' to {}".format(style_filename, user_source_view_style_path))
                theme_source_view_style_path = os.path.join(theme_source_view_path, style_filename)
                shutil.copy(theme_source_view_style_path, user_source_view_style_path)
    except IOError as e:
        log.error("Could not install GTKSourceView style: {}".format(e))


def install_libraries():
    if glib:
        user_data_folder = glib.get_user_data_dir()
    else:
        user_data_folder = os.path.join(os.path.expanduser('~'), '.local', 'share')
    user_library_path = os.path.join(user_data_folder, 'rafcon', 'libraries')
    library_path = os.path.join("share", "libraries")

    if os.path.exists(user_library_path):
        try:
            log.info("Removing old RAFCON libraries in {}".format(user_library_path))
            shutil.rmtree(user_library_path)
        except (EnvironmentError, shutil.Error) as e:
            log.error("Could not remove old RAFCON libraries in {}: {}".format(user_library_path, e))
            return

    try:
        log.info("Installing RAFCON libraries to {}".format(user_library_path))
        shutil.copytree(library_path, user_library_path)
    except (IOError, shutil.Error):
        log.error("Could not install RAFCON libraries: {}".format(e))


class PostDevelopCommand(DevelopCommand):
    """Post-installation for development mode."""
    def run(self):
        install_fonts()
        install_gtk_source_view_styles()
        install_libraries()
        DevelopCommand.run(self)


class PostInstallCommand(InstallCommand):
    """Post-installation for installation mode."""
    def run(self):
        install_fonts()
        install_gtk_source_view_styles()
        install_libraries()
        InstallCommand.run(self)


def get_data_files_tuple(*path, **kwargs):
    """Return a tuple which can be used for setup.py's data_files
    
    :param tuple path: List of path elements pointing to a file or a directory of files
    :param dict kwargs: Set path_to_file to True is `path` points to a file 
    :return: tuple of install directory and list of source files
    :rtype: tuple(str, [str])
    """
    path = os.path.join(*path)
    target_path = os.path.join(*path.split(os.sep)[1:])  # remove source/ (package_dir)
    if "path_to_file" in kwargs and kwargs["path_to_file"]:
        source_files = [path]
        target_path = os.path.dirname(target_path)
    else:
        source_files = [os.path.join(path, filename) for filename in os.listdir(path)]
    return target_path, source_files


global_requirements = ['astroid', 'pylint', 'pyyaml', 'psutil', 'jsonconversion~=0.2', 'yaml_configuration~=0.0',
                       'python-gtkmvc-dlr==1.99.2', 'gaphas>=0.7']

assets_folder = os.path.join('source', 'rafcon', 'gui', 'assets')
themes_folder = os.path.join(assets_folder, 'themes')

# read version from VERSION file
# this might throw Exceptions, which are purposefully not caught as the version is a prerequisite for installing rafcon
version_file_path = os.path.join(os.path.dirname(__file__), "VERSION")
with open(version_file_path, "r") as f:
    content = f.read().splitlines()
    version = content[0]

setup(
    name='rafcon',
    version=version,
    url='https://github.com/DLR-RM/RAFCON',
    license='EPL',
    author='Sebastian Brunner, Rico Belder, Franz Steinmetz',
    author_email='sebastian.brunner@dlr.de, rico.belder@dlr.de, franz.steinmetz@dlr.de',
    description='Develop your robotic tasks with hierarchical state machines using an intuitive graphical user '
                'interface',

    packages=find_packages('source'),
    package_dir={'': 'source'},  # tell distutils packages are under src

    package_data={
        # Include config files
        'rafcon': ['pylintrc'],
        # Include core and GUI config
        'rafcon.core': ['config.yaml'],
        'rafcon.gui': ['gui_config.yaml'],
        # Include all glade files
        'rafcon.gui.glade': ['*.glade']
    },

    data_files=[
        get_data_files_tuple(assets_folder, 'icons'),
        get_data_files_tuple(assets_folder, 'splashscreens'),
        get_data_files_tuple(themes_folder, 'dark', 'gtk-2.0', 'gtkrc', path_to_file=True),
        get_data_files_tuple(themes_folder, 'dark', 'colors.json', path_to_file=True),
        get_data_files_tuple(themes_folder, 'dark', 'gtk-sourceview'),
    ],

    setup_requires=['Sphinx>=1.4', 'Pygments>=2.0'] + global_requirements,
    tests_require=['pytest', 'pytest-catchlog'] + global_requirements,
    install_requires=global_requirements,

    dependency_links=[
        "https://github.com/DLR-RM/gtkmvc3/releases/download/gtkmvc_dlr_1.99.2/python-gtkmvc-dlr-1.99.2.tar.gz"
        "#egg=python-gtkmvc-dlr-1.99.2"
    ],

    entry_points={
        'console_scripts': [
            'rafcon_start = rafcon.core.start:main'
        ],
        'gui_scripts': [
            'rafcon_start_gui = rafcon.gui.start:main'
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
