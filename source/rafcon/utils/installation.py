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
from os import path
from os.path import dirname
import sys
import shutil
import subprocess
import distutils.log

try:
    from gi.repository import Gtk
except ImportError:
    Gtk = None
try:
    from gi.repository import GLib
except ImportError:
    GLib = None

# Path to this file: source/rafcon/utils/installation.py
rafcon_root_path = dirname(dirname(dirname(dirname(path.abspath(__file__)))))
assets_folder = os.path.join('source', 'rafcon', 'gui', 'assets')


def install_fonts(logger=None, restart=False):
    if logger:
        log = logger
    else:
        log = distutils.log
    if not Gtk:
        log.warn("No GTK found. Will not install fonts.")
        return

    tv = Gtk.TextView()
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

    font_installed = False
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
            font_installed = True
    except IOError as e:
        log.error("Could not install fonts, IOError: {}".format(e))
        return

    if font_installed:
        log.info("Running font detection ...")
        fail = subprocess.call(['fc-cache', '-fv', '"' + user_otf_fonts_folder + '"'])
        if fail:
            log.warn("Could not run font detection. RAFCON might not find the correct fonts.")
        if restart:
            log.info("Restarting RAFCON to apply new fonts...")
            python = sys.executable
            environ = dict(**os.environ)
            # Passing this to the new RAFCON environment will prevent further checks and thus restarts
            environ["RAFCON_CHECK_INSTALLATION"] = "False"
            args_and_env = list(sys.argv)
            args_and_env.append(environ)
            os.execle(python, python, *args_and_env)


def install_gtk_source_view_styles(logger=None):
    if logger:
        log = logger
    else:
        log = distutils.log
    if GLib:
        user_data_folder = GLib.get_user_data_dir()
    else:
        user_data_folder = os.path.join(os.path.expanduser('~'), '.local', 'share')
    user_source_view_style_path = os.path.join(user_data_folder, 'gtksourceview-3.0', 'styles')

    try:
        if not os.path.exists(user_source_view_style_path):
            os.makedirs(user_source_view_style_path)

        # Copy all .xml source view style files from all themes to local user styles folder
        themes_path = os.path.join(rafcon_root_path, assets_folder, "share", "themes")
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


def install_libraries(logger=None, overwrite=True):
    if logger:
        log = logger
    else:
        log = distutils.log
    if GLib:
        user_data_folder = GLib.get_user_data_dir()
    else:
        user_data_folder = os.path.join(os.path.expanduser('~'), '.local', 'share')
    user_library_path = os.path.join(user_data_folder, 'rafcon', 'libraries')
    library_path = os.path.join(rafcon_root_path, "share", "libraries")

    if os.path.exists(user_library_path):
        if not overwrite:
            return
        try:
            log.info("Removing old RAFCON libraries in {}".format(user_library_path))
            shutil.rmtree(user_library_path)
        except (EnvironmentError, shutil.Error) as e:
            log.error("Could not remove old RAFCON libraries in {}: {}".format(user_library_path, e))
            return

    try:
        log.info("Installing RAFCON libraries to {}".format(user_library_path))
        shutil.copytree(library_path, user_library_path)
    except (IOError, shutil.Error) as e:
        log.error("Could not install RAFCON libraries: {}".format(e))


def create_mo_files():
    from os import path
    import subprocess
    data_files = []
    domain = "rafcon"
    rel_localedir = path.join('source', 'rafcon', 'locale')
    localedir = os.path.join(rafcon_root_path, rel_localedir)
    po_files = [po_file
                for po_file in next(os.walk(localedir))[2]
                if path.splitext(po_file)[1] == '.po']
    for po_file in po_files:
        po_path = path.join(localedir, po_file)
        lang, extension = path.splitext(po_file)
        mo_file = domain + '.mo'
        mo_dir = path.join(localedir, lang, 'LC_MESSAGES')
        mo_path = path.join(mo_dir, mo_file)
        mo_dir_rel = path.join(rel_localedir, lang, 'LC_MESSAGES')
        mo_path_rel = path.join(mo_dir_rel, mo_file)
        try:
            os.makedirs(mo_dir)
        except os.error:  # already exists
            pass
        msgfmt_cmd = 'msgfmt -o {} {}'.format(mo_path, po_path)
        result = subprocess.call(msgfmt_cmd, shell=True)
        if result == 0:  # Compilation successful
            # add po file
            target_dir = path.join("share", *rel_localedir.split(os.sep)[1:])  # remove source/ (package_dir)
            data_files.append((target_dir, [os.path.join(rel_localedir, po_file)]))
            # add mo file
            target_dir = path.join("share", *mo_dir_rel.split(os.sep)[1:])  # remove source/ (package_dir)
            data_files.append((target_dir, [mo_path_rel]))
        else:
            distutils.log.warn("Could not compile translation '{}'. RAFCON will not be available in this "
                               "language.".format(lang))

    return data_files


def get_data_files_tuple(*rel_path, **kwargs):
    """Return a tuple which can be used for setup.py's data_files

    :param tuple path: List of path elements pointing to a file or a directory of files
    :param dict kwargs: Set path_to_file to True is `path` points to a file
    :return: tuple of install directory and list of source files
    :rtype: tuple(str, [str])
    """
    rel_path = os.path.join(*rel_path)
    target_path = os.path.join("share", *rel_path.split(os.sep)[1:])  # remove source/ (package_dir)
    if "path_to_file" in kwargs and kwargs["path_to_file"]:
        source_files = [rel_path]
        target_path = os.path.dirname(target_path)
    else:
        source_files = [os.path.join(rel_path, filename) for filename in os.listdir(rel_path)]
    return target_path, source_files


def get_data_files_recursively(*rel_root_path, **kwargs):
    """ Adds all files of the specified path to a data_files compatible list

    :param tuple rel_root_path: List of path elements pointing to a directory of files
    :return: list of tuples of install directory and list of source files
    :rtype: list(tuple(str, [str]))
    """
    result_list = list()
    rel_root_dir = os.path.join(*rel_root_path)
    share_target_root = os.path.join("share", kwargs.get("share_target_root", "rafcon"))
    distutils.log.debug("recursively generating data files for folder '{}' ...".format(
        rel_root_dir))

    for dir_, _, files in os.walk(rel_root_dir):
        relative_directory = os.path.relpath(dir_, rel_root_dir)
        file_list = list()
        for fileName in files:
            rel_file_path = os.path.join(relative_directory, fileName)
            abs_file_path = os.path.join(rel_root_dir, rel_file_path)
            file_list.append(abs_file_path)
        if len(file_list) > 0:
            # this is a valid path in ~/.local folder: e.g. share/rafcon/libraries/generic/wait
            target_path = os.path.join(share_target_root, relative_directory)
            result_list.append((target_path, file_list))
    return result_list


def generate_data_files():
    """ Generate the data_files list used in the setup function

    :return: list of tuples of install directory and list of source files
    :rtype: list(tuple(str, [str]))
    """
    assets_folder = path.join('source', 'rafcon', 'gui', 'assets')
    share_folder = path.join(assets_folder, 'share')
    themes_folder = path.join(share_folder, 'themes', 'RAFCON')
    examples_folder = path.join('share', 'examples')
    libraries_folder = path.join('share', 'libraries')

    gui_data_files = [
        get_data_files_tuple(assets_folder, 'splashscreens'),
        get_data_files_tuple(assets_folder, 'fonts', 'FontAwesome'),
        get_data_files_tuple(assets_folder, 'fonts', 'DIN Next LT Pro'),
        get_data_files_tuple(themes_folder, 'gtk-3.0', 'gtk.css', path_to_file=True),
        get_data_files_tuple(themes_folder, 'gtk-3.0', 'gtk-dark.css', path_to_file=True),
        get_data_files_tuple(themes_folder, 'assets'),
        get_data_files_tuple(themes_folder, 'sass'),
        get_data_files_tuple(themes_folder, 'gtk-sourceview'),
        get_data_files_tuple(themes_folder, 'colors.json', path_to_file=True),
        get_data_files_tuple(themes_folder, 'colors-dark.json', path_to_file=True)
    ]
    # print("gui_data_files", gui_data_files)

    icon_data_files = get_data_files_recursively(path.join(share_folder, 'icons'), share_target_root="icons")
    # print("icon_data_files", icon_data_files)

    locale_data_files = create_mo_files()
    # example tuple
    # locale_data_files = [('share/rafcon/locale/de/LC_MESSAGES', ['source/rafcon/locale/de/LC_MESSAGES/rafcon.mo'])]
    # print("locale_data_files", locale_data_files)

    version_data_file = [("./", ["VERSION"])]
    desktop_data_file = [("share/applications", [path.join('share', 'applications', 'de.dlr.rm.RAFCON.desktop')])]

    examples_data_files = get_data_files_recursively(examples_folder, share_target_root=path.join("rafcon", "examples"))
    libraries_data_files = get_data_files_recursively(libraries_folder, share_target_root=path.join("rafcon",
                                                                                                    "libraries"))
    generated_data_files = gui_data_files + icon_data_files + locale_data_files + version_data_file + \
                           desktop_data_file + examples_data_files + libraries_data_files
    # for elem in generated_data_files:
    #     print(elem)
    return generated_data_files
