#!/opt/python/python2.7/bin/python

import logging
import os
import gtk
import signal
import argparse
from os.path import realpath, dirname, join, exists, expanduser, expandvars, isdir
import threading
import sys

import rafcon

from rafcon.utils import log

from rafcon.statemachine.config import global_config
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
import rafcon.statemachine.singleton as sm_singletons
import rafcon.statemachine.storage.storage as storage

from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
import rafcon.mvc.singleton as mvc_singletons
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config


def setup_logger():
    import sys
    import logging

    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)


def call_gui_callback(callback, *args):
    import glib
    import threading
    condition = threading.Condition()

    def fun():
        """Call callback and notify condition variable
        """
        try:
            callback(*args)
        finally:  # Finally is also executed in the case of exceptions and reraises the exception at the end
            condition.acquire()
            condition.notify()
            condition.release()

    glib.idle_add(fun)
    # Wait for the condition to be notified
    condition.acquire()
    condition.wait()
    condition.release()


def trigger_gui_signals(*args):
    print "Triggering save as"
    main_window_controller = args[1]
    setup_config = args[2]
    state_machine = args[3]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    menubar_ctrl.on_save_as_activate(None, None, setup_config['target_path'][0])
    import time
    while state_machine.marked_dirty:
        time.sleep(0.1)
    call_gui_callback(menubar_ctrl.on_stop_activate, None)
    call_gui_callback(menubar_ctrl.on_quit_activate, None)


def convert(config_path, source_path, target_path):
    setup_logger()
    # from rafcon.utils import log
    logger = log.get_logger("start")
    logger.info("RAFCON launcher")

    rafcon_root_path = dirname(realpath(rafcon.__file__))
    if not os.environ.get('RAFCON_PATH', None):
        # set env variable RAFCON_PATH to the root directory of RAFCON
        os.environ['RAFCON_PATH'] = rafcon_root_path

    if not os.environ.get('RAFCON_LIB_PATH', None):
        # set env variable RAFCON_LIB_PATH to the library directory of RAFCON (when not using RMPM)
        os.environ['RAFCON_LIB_PATH'] = join(dirname(rafcon_root_path), 'libraries')

    home_path = expanduser('~')
    if home_path:
        home_path = join(home_path, ".config", "rafcon")
    else:
        home_path = 'None'

    setup_config = {}
    setup_config["config_path"] = config_path
    setup_config["gui_config_path"] = home_path
    setup_config["source_path"] = [source_path]
    if not target_path:
        setup_config["target_path"] = [source_path]
    else:
        setup_config["target_path"] = [target_path]

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(path=setup_config['config_path'])
    global_gui_config.load(path=setup_config['gui_config_path'])
    global_runtime_config.load(path=setup_config['gui_config_path'])

    # Make mvc directory the working directory
    # Needed for views, which assume to be in the mvc path and import glade files relatively
    os.chdir(join(rafcon_root_path, 'mvc'))

    # Initialize library
    sm_singletons.library_manager.initialize()

    # Create the GUI
    main_window_view = MainWindowView()

    if setup_config['source_path']:
        if len(setup_config['source_path']) > 1:
            logger.error("Only one statemachine is supported yet")
            exit(-1)
        for path in setup_config['source_path']:
            try:
                state_machine = storage.load_statemachine_from_path(path)
                sm_singletons.state_machine_manager.add_state_machine(state_machine)
            except Exception as e:
                logger.error("Could not load state-machine {0}: {1}".format(path, e))
    else:
        logger.error("You need to specify exactly one state machine to be converted!")

    sm_manager_model = mvc_singletons.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type="LogicDataGrouped")
    main_window = main_window_view.get_top_widget()
    size = global_runtime_config.get_config_value("WINDOW_SIZE", None)
    position = global_runtime_config.get_config_value("WINDOW_POS", None)
    if size:
        main_window.resize(size[0], size[1])
    if position:
        position = (max(0, position[0]), max(0, position[1]))
        screen_width = gtk.gdk.screen_width()
        screen_height = gtk.gdk.screen_height()
        if position[0] < screen_width and position[1] < screen_height:
            main_window.move(position[0], position[1])

    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)
    thread = threading.Thread(target=trigger_gui_signals, args=[sm_manager_model,
                                                                main_window_controller,
                                                                setup_config,
                                                                state_machine])
    thread.start()

    gtk.main()
    logger.debug("Gtk main loop exited!")
    logger.debug("Conversion done")


def convert_libraries_in_path(config_path, lib_path, target_path=None):
    """
    This function resaves all libraries found at the spcified path
    :param lib_path: the path to look for libraries
    :return:
    """
    for lib in os.listdir(lib_path):
        if os.path.isdir(os.path.join(lib_path, lib)) and not '.' == lib[0]:
            if os.path.exists(os.path.join(os.path.join(lib_path, lib), "statemachine.yaml")) or \
                    os.path.exists(os.path.join(os.path.join(lib_path, lib), "statemachine.json")):
                print "convert " + os.path.join(lib_path, lib)
                if not target_path:
                    convert(config_path, os.path.join(lib_path, lib))
                else:
                    convert(config_path, os.path.join(lib_path, lib), os.path.join(target_path, lib))
            else:
                if not target_path:
                    convert_libraries_in_path(config_path, os.path.join(lib_path, lib))
                else:
                    convert_libraries_in_path(config_path, os.path.join(lib_path, lib), os.path.join(target_path, lib))
        else:
            if os.path.isdir(os.path.join(lib_path, lib)) and '.' == lib[0]:
                print "lib_root_path/lib_path .*-folder are ignored if within lib_path, e.g. -> {0} -> full path is {1}".format(lib, os.path.join(lib_path, lib))


if __name__ == '__main__':
    import sys
    folder_to_convert = sys.argv[2]
    config_path = sys.argv[1]
    print "folder to convert: " + folder_to_convert
    convert_libraries_in_path(config_path, folder_to_convert)
