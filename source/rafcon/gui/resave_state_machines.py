# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

#!/usr/bin/python

from future import standard_library
standard_library.install_aliases()
import os
from os.path import join, expanduser
import threading
import time

from rafcon.utils import log
from rafcon.utils.filesystem import get_default_config_path
from rafcon.utils.gui_functions import call_gui_callback

from rafcon.core.config import global_config
import rafcon.core.singleton as core_singletons

import rafcon.gui.start
import rafcon.gui.singleton as gui_singletons
from rafcon.gui.utils import wait_for_gui
from rafcon.gui.config import global_gui_config
from rafcon.gui.runtime_config import global_runtime_config
import rafcon.gui.helpers.state_machine as gui_helper_state_machine
from rafcon.gui.utils.gtk_utils import is_point_on_screen
from rafcon.core.start import setup_environment


def setup_logger():
    import sys
    import logging

    # Apply defaults to logger of gtkmvc3
    for handler in logging.getLogger('gtkmvc3').handlers:
        logging.getLogger('gtkmvc3').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc3').addHandler(stdout)


setup_logger()
logger = log.get_logger("Resave state machines script")


def trigger_gui_signals(*args):
    sm_manager_model = args[0]
    main_window_controller = args[1]
    setup_config = args[2]
    state_machine = args[3]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    try:
        sm_manager_model.selected_state_machine_id = state_machine.state_machine_id
        call_gui_callback(gui_helper_state_machine.save_state_machine_as, path=setup_config['target_path'][0])
    except:
        logger.exception("Could not save state machine")
    finally:
        call_gui_callback(menubar_ctrl.on_quit_activate, None)


def convert(config_path, source_path, target_path=None, gui_config_path=None):
    logger.info("RAFCON launcher")
    rafcon.gui.start.setup_l10n(logger)

    from gi.repository import Gtk
    from rafcon.gui.controllers.main_window import MainWindowController
    from rafcon.gui.views.main_window import MainWindowView

    setup_environment()

    gui_config_path = gui_config_path or get_default_config_path()

    setup_config = {}
    setup_config["config_path"] = config_path
    setup_config["gui_config_path"] = gui_config_path
    setup_config["source_path"] = [source_path]
    if not target_path:
        setup_config["target_path"] = [source_path]
    else:
        setup_config["target_path"] = [target_path]

    global_config.load(path=setup_config['config_path'])
    global_gui_config.load(path=setup_config['gui_config_path'])
    global_runtime_config.load(path=setup_config['gui_config_path'])

    # Initialize library
    core_singletons.library_manager.initialize()

    # Create the GUI
    main_window_view = MainWindowView()

    if setup_config['source_path']:
        if len(setup_config['source_path']) > 1:
            logger.error("Only one state machine is supported yet")
            exit(-1)
        for path in setup_config['source_path']:
            try:
                state_machine = gui_helper_state_machine.open_state_machine(path)
            except Exception as e:
                logger.error("Could not load state machine {0}: {1}".format(path, e))
    else:
        logger.error("You need to specify exactly one state machine to be converted!")

    sm_manager_model = gui_singletons.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view)

    if not os.getenv("RAFCON_START_MINIMIZED", False):
        main_window = main_window_view.get_top_widget()
        size = global_runtime_config.get_config_value("WINDOW_SIZE", None)
        position = global_runtime_config.get_config_value("WINDOW_POS", None)
        if size:
            main_window.resize(size[0], size[1])
        if position:
            position = (max(0, position[0]), max(0, position[1]))
            if is_point_on_screen(*position):
                main_window.move(*position)

    wait_for_gui()
    thread = threading.Thread(target=trigger_gui_signals, args=[sm_manager_model,
                                                                main_window_controller,
                                                                setup_config,
                                                                state_machine])
    thread.start()

    Gtk.main()
    logger.debug("Gtk main loop exited!")
    logger.debug("Conversion done")


def convert_libraries_in_path(config_path, lib_path, target_path=None, gui_config_path=None):
    """
    This function resaves all libraries found at the spcified path
    :param lib_path: the path to look for libraries
    :return:
    """
    for lib in os.listdir(lib_path):
        child_lib_path = os.path.join(lib_path, lib)
        if os.path.isdir(child_lib_path) and not '.' == lib[0]:
            lib_target_path = None if not target_path else os.path.join(target_path, lib)
            if os.path.exists(os.path.join(child_lib_path, "statemachine.yaml")) or \
                    os.path.exists(os.path.join(child_lib_path, "statemachine.json")):
                convert(config_path, child_lib_path, lib_target_path, gui_config_path)
            else:
                convert_libraries_in_path(config_path, child_lib_path, lib_target_path, gui_config_path)
        else:
            if os.path.isdir(child_lib_path) and '.' == lib[0]:
                logger.debug("lib_root_path/lib_path .*-folder are ignored if within lib_path, "
                             "e.g. -> {0} -> full path is {1}".format(lib, child_lib_path))

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 3:
        logger.error("Wrong number of arguments")
        logger.error("Usage: resave_state_machine.py config_path library_folder_to_convert optional_target_folder gui_config_path")
        exit(0)
    config_path = sys.argv[1]
    gui_config_path = None if len(sys.argv) < 5 else sys.argv[4]
    folder_to_convert = sys.argv[2]
    target_path = None if len(sys.argv) < 4 else sys.argv[3]
    logger.info("folder to convert: " + folder_to_convert)
    convert_libraries_in_path(config_path, folder_to_convert, target_path, gui_config_path)
