from conans import ConanFile, CMake, tools
import os
import subprocess
import arpm


# read version from VERSION file
# this might throw Exceptions, which are purposefully not caught as the version is a prerequisite for installing rafcon
# version_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
#                                  "export_source", "VERSION")

# version_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "VERSION")
# with open(version_file_path, "r") as f:
#     content = f.read().splitlines()
#     rafcon_version = content[0]

# with open("VERSION", "r") as f:
#     content = f.read().splitlines()
#     rafcon_version = content[0]


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

    options = {"python3": [True, False],
               }
    # setting python3 to False currently does not work!
    default_options = {"python3": True,
                       }

    settings = 'os', 'compiler', 'build_type', 'arch'
    exports_sources = "VERSION"

    def build(self):
        envd = dict(os.environ)
        # Check if a virtual environment is active
        # or a PYTHONPATH is set, we shouldn't do anything
        # because the set of 'system packages' might be different.
        if 'VIRTUAL_ENV' in envd or 'PYTHONHOME' in envd:
            raise RuntimeError(
                "Cannot create package with an active virtual environment. VIRTUAL_ENV or PYTHONHOME is set!")
        envd['PYTHONUSERBASE'] = self.package_folder

        # create pip package in dist
        subprocess.run(['python3', 'setup.py', 'sdist', 'bdist_wheel'])
        print("Installing rafcon for python3")
        print("Package folder: {}".format(str(self.package_folder)))
        subprocess.run(['pip3', 'install', './dist/rafcon-{}.tar.gz'.format(self.version)], env=envd)

        # currently, there are different errors when trying to either running the setup.py with py2
        # or using pip2 to install the tar.gz created with python3
        # print("Installing rafcon for python2")
        # subprocess.run(['pip2', 'install', './dist/rafcon-{}.tar.gz'.format(self.version)], env=envd)

    def package_info(self):
        if self.options.python3:
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "lib", "python3.6", "site-packages"))
        else:  # python2
            self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, "lib", "python2.7", "site-packages"))
        self.env_info.XDG_DATA_HOME = os.path.join(self.package_folder, "share")
        self.env_info.PATH.append(os.path.join(self.package_folder, "bin"))
        self.env_info.RAFCON_ROOT_DIRECTORY = self.package_folder
