from conans import ConanFile, CMake, tools
import os
import subprocess
import arpm


def get_version():
    try:
        # see https://docs.conan.io/en/1.4/howtos/capture_version.html
        content = tools.load("VERSION")
        return content
    except FileNotFoundError as e:
        return None


class RafconConan(ConanFile):
    name = "rafcon"
    version = get_version()
    license = "Eclipse Public License 1.0"
    author = ('Sebastian Brunner <sebastian.brunner@dlr.de>,'
              'Rico Belder <rico.belder@dlr.de>,'
              'Franz Steinmetz<franz.steinmetz@dlr.de')
    description = "Develop your robotic tasks with hierarchical state machines" \
                  "using an intuitive graphical user interface."

    url = 'http://git.ar.int/dev/sys/tci/rafcon'
    scm = {
        "revision": "auto",
        "type": "git",
        "url": "git@git.ar.int:dev/sys/tci/rafcon.git"
    }

    default_user = "ar"
    default_channel = "stable"

    options = {
        "python3": [True, False],
    }

    default_options = {
        "python3": True,
    }

    settings = 'os', 'compiler', 'build_type', 'arch'
    exports_sources = "VERSION"

    def build(self):
        envd = dict(os.environ)
        # Check if a virtual environment is active or a PYTHONPATH is set
        # in this case, we shouldn't do anything because the set of 'system packages' might be different.
        if 'VIRTUAL_ENV' in envd or 'PYTHONHOME' in envd:
            raise RuntimeError(
                "Cannot create package with an active virtual environment. VIRTUAL_ENV or PYTHONHOME is set!")
        envd['PYTHONUSERBASE'] = self.package_folder

        # create pip package in dist
        # can be used for python2 and python3
        subprocess.run(['python3', 'setup.py', 'sdist', 'bdist_wheel'])
        print("Package folder: {}".format(str(self.package_folder)))
        print("Installing rafcon for python3")
        # --ignore-installed is required as otherwise already installed packages
        # would prevent pip from installing required dependencies
        # this is also true for the packages introduced by our local conan environment in /opt/conan/lib/python3.6/
        subprocess.run(['pip3', 'install', '--ignore-installed', './dist/rafcon-{}.tar.gz'.format(
            self.version)], env=envd)

        # print("Installing pip first")
        # subprocess.run(['python2.7', '-m', 'pip', 'install', '--user', '--upgrade', '--force', 'pip'])
        # print("Installing setuptools first")
        # subprocess.run(['python2.7', '-m', 'pip', 'install', '--user', '--upgrade', 'setuptools==44.1.1'])

        print("Installing rafcon for python2")
        # using pip2 to install the tar.gz does not work unfortunately
        # error: 'egg_base' must be a directory name (got `/tmp/pip-modern-metadata-S8oLCh`)
        # subprocess.run(['pip2', 'install', './dist/rafcon-{}.tar.gz'.format(self.version)], env=envd)
        subprocess.run(['pip2', 'install', '--ignore-installed', '--user', './dist/rafcon-{}-py2.py3-none-any.whl'.format(self.version)], env=envd)

    def package_info(self):
        if self.options.python3:
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "lib", "python3.6", "site-packages"))
        else:  # python2
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "lib", "python2.7", "site-packages"))
        self.env_info.XDG_DATA_HOME = os.path.join(self.package_folder, "share")
        self.env_info.PATH.append(os.path.join(self.package_folder, "bin"))
        self.env_info.RAFCON_ROOT_DIRECTORY = self.package_folder
