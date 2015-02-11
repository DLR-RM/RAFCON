This folder should containt two script files:
* sys_lib_install.sh installs all systems dependencies and then calls the second script
* python_lib_install.sh installs all python dependencies 

You have two define, in which folders the system and python dependencies should be installed. The two paths are
the first and seconds argument of the sys script:

sys_lib_install.sh /path/to/sys/dependencies/ /path/to/python/dependencies

If the system dependencies have already been installed, you can also just run the python script:

python_lib_install.sh /path/to/python/dependencies

With all dependencies installed, you manually have to set up your environment. E. g. include the following
lines in your ~/.bashrc file:

# Setup ROS Indigo
source /volume/USERSTORE/lehn_pt/ros/indigo/setup.bash
# Setup Python path
python_dep=/path/to/python/dependencies
export PYTHONPATH=$PYTHONPATH:$python_dep:$python_dep/lib/python2.7/site-packages/:$python_dep/lib/python2.7/site-packages/gtk-2.0
# Setup system library path
sys_dep=/path/to/system/dependencies
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$sys_dep/lib:$sys_dep/lib64/'

Good luck ;-)
