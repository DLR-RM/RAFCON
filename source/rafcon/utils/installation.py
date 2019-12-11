# Copyright (C) 2017-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import os
from os.path import join, dirname, abspath, isfile, isdir
import sys
import subprocess
from distutils.dir_util import copy_tree

from rafcon.utils import resources, log

logger = log.get_logger(__name__)


def started_without_installation():
    import rafcon
    pt_file_path = join(dirname(dirname(dirname(abspath(rafcon.__file__)))), "rafcon.pt")
    if isfile(pt_file_path):
        return True


def started_in_virtualenv():
    return isdir(os.getenv("VIRTUAL_ENV", ""))


def update_font_cache(path):
    try:
        fail = subprocess.call(['fc-cache', path])
        return not fail
    except OSError:
        return False


def installed_font_faces_for_font(font_name):
    try:
        p = subprocess.Popen(['fc-list', font_name], stdout=subprocess.PIPE)
        output, _ = p.communicate()
        output_lines = str(output.decode("utf-8")).split("\n")
        return len([line for line in output_lines if font_name in line])
    except OSError:
        return 0


def install_fonts(restart=False):
    # do not import from rafcon.gui.constants, as this script can be used standalone
    font_names_to_be_installed = ["SourceSansPro", "FontAwesome5Free", "RAFCON"]

    user_otf_fonts_folder = join(resources.xdg_user_data_folder, "fonts")

    font_installed = False
    try:
        for font_name in font_names_to_be_installed:
            # A font is a folder one or more font faces
            rel_font_folder = join("type1", font_name)
            fonts_folder = resources.get_data_file_path("fonts", rel_font_folder)
            num_faces_to_be_installed = len([name for name in os.listdir(fonts_folder) if name.endswith(".otf")])
            num_faces_installed = installed_font_faces_for_font(font_name)

            if num_faces_to_be_installed <= num_faces_installed:
                logger.debug("Font '{0}' already installed".format(font_name))
                continue

            specific_user_otf_fonts_folder = join(user_otf_fonts_folder, rel_font_folder)
            logger.info("Installing font '{0}' to {1}".format(font_name, specific_user_otf_fonts_folder))
            copy_tree(fonts_folder, specific_user_otf_fonts_folder, update=1)
            font_installed = True
    except IOError as e:
        logger.error("Could not install fonts, IOError: {}".format(e))
        return

    if font_installed:
        logger.info("Running font detection ...")
        if not update_font_cache(user_otf_fonts_folder):
            logger.warning("Could not run font detection using 'fc-cache'. RAFCON might not find the correct fonts.")
        if restart:
            python = sys.executable
            environ = dict(**os.environ)
            # Passing this to the new RAFCON environment will prevent further checks and thus restarts
            environ["RAFCON_CHECK_INSTALLATION"] = "False"
            args_and_env = list(sys.argv)
            args_and_env.append(environ)
            logger.info("Restarting RAFCON ...")
            os.execle(python, python, *args_and_env)


def install_locally_required_files():
    source_share_folder = resources.get_data_file_path()
    if not source_share_folder:
        logger.warning("Cannot find repository required for installation of icons and gtksourceview styles")

    for folder in ["gtksourceview-3.0", "icons"]:
        try:
            logger.info("Copying '{}' files...".format(folder))
            copy_tree(join(source_share_folder, folder), join(resources.xdg_user_data_folder, folder), update=1)
        except IOError as e:
            logger.error("Could not copy '{}' files: {}".format(folder, str(e)))
