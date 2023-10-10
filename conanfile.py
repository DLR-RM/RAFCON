import os
import subprocess
import sys

from conans import CMake, ConanFile, tools


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
    }

    default_options = {
    }

    settings = 'os', 'compiler', 'build_type', 'arch'
    exports_sources = "VERSION"

    def system_requirements(self):
        import pip
        if hasattr(pip, "main"):
            pip.main(["install", "pdm"])
        else:
            from pip._internal import main
            main(['install', "pdm"])

    def build(self):
        envd = dict(os.environ)
        # Check if a virtual environment is active or a PYTHONPATH is set
        # in this case, we shouldn't do anything because the set of 'system packages' might be different.
        if 'VIRTUAL_ENV' in envd or 'PYTHONHOME' in envd:
            raise RuntimeError(
                "Cannot create package with an active virtual environment. VIRTUAL_ENV or PYTHONHOME is set!")
        envd['PYTHONUSERBASE'] = self.package_folder
        envd['PYTHONPATH'] = ""  # unset PYTHONPATH in order to not get conflicts with exiting packages
        # print("envd: {}".format(str(envd)))
        # print("Package folder: {}".format(str(self.package_folder)))

        self.run("pdm install --no-editable")
        self.run("pdm build")

        print("Installing rafcon for python3")

        rafcon_target = './dist/rafcon-{}.tar.gz'.format(self.version)

        # --ignore-installed is required as otherwise already installed packages
        # would prevent pip from installing required dependencies
        # this is also true for the packages introduced by our local conan environment in /opt/conan/lib/python3.6/
        # https://stackoverflow.com/questions/51913361/difference-between-pip-install-options-ignore-installed-and-force-reinstall
        subprocess.run([
            'python3', '-m', 'pip', 'install', '--user', '--ignore-installed', rafcon_target
        ], env=envd, check=True)

    def package_info(self):
        site_packages = os.path.join(self.package_folder, "lib", "python3.10", "site-packages")
        self.env_info.PYTHONPATH.append(site_packages)
        python_version = sys.version_info
        self.env_info.XDG_DATA_HOME = os.path.join(self.package_folder, "lib", f"python{python_version.major}.{python_version.minor}", "site-packages", "rafcon", "share")
        self.env_info.PATH.append(os.path.join(self.package_folder, "bin"))
        self.env_info.RAFCON_ROOT_DIRECTORY = self.package_folder
