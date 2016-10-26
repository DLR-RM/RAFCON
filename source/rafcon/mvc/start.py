#!/usr/bin/env python

import os
import logging
import gtk
import threading

import signal

from yaml_configuration.config import config_path

from rafcon.statemachine.start import parse_state_machine_path, setup_environment, reactor_required, \
    setup_configuration, post_setup_plugins, register_signal_handlers
from rafcon.statemachine.storage import storage
from rafcon.statemachine.state_machine import StateMachine
from rafcon.statemachine.states.hierarchy_state import HierarchyState
import rafcon.statemachine.singleton as sm_singletons
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.statemachine.config import global_config

import rafcon.mvc.singleton as mvc_singletons
from rafcon.mvc.controllers.main_window import MainWindowController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.config import global_gui_config
from rafcon.mvc.runtime_config import global_runtime_config
from rafcon.mvc.utils import constants

import rafcon.utils.filesystem as filesystem
from rafcon.utils import profiler
from rafcon.utils import plugins
from rafcon.utils.constants import RAFCON_TEMP_PATH_BASE

from rafcon.utils.i18n import _, setup_l10n, setup_l10n_gtk

from rafcon.utils import log

logger = log.get_logger("rafcon.start.gui")


def setup_gtkmvc_logger():
    import sys
    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)


def pre_setup_plugins():
    """Loads plugins and calls the pre init hooks

    If twisted has been imported by a plugin, the gtk2reactor is installed
    """
    # load all plugins specified in the RAFCON_PLUGIN_PATH
    plugins.load_plugins()

    # check if twisted is imported and if so, install the required reactor
    if reactor_required():
        from twisted.internet import gtk2reactor
        # needed for glib.idle_add, and signals
        gtk2reactor.install()

    plugins.run_pre_inits()


def setup_mvc_environment():
    setup_environment()


def start_state_machine(state_machine, start_state_path, quit_flag):
    sm_thread = threading.Thread(target=start_stop_state_machine,
                                 args=[state_machine, start_state_path, quit_flag])
    sm_thread.start()


def start_stop_state_machine(state_machine, start_state_path, quit_flag):
    # Wait for GUI to initialize
    while gtk.events_pending():
        gtk.main_iteration(False)

    state_machine_execution_engine = sm_singletons.state_machine_execution_engine
    state_machine_execution_engine.execute_state_machine_from_path(state_machine=state_machine,
                                                                   start_state_path=start_state_path,
                                                                   wait_for_execution_finished=True)
    if reactor_required():
        from twisted.internet import reactor
        reactor.callFromThread(reactor.stop)

    if quit_flag:
        mvc_singletons.main_window_controller.get_controller('menu_bar_controller').on_quit_activate(None, None)


def setup_argument_parser():
    """Sets up teh parser with the required arguments

    :return: The parser object
    """
    home_path = filesystem.get_home_path()

    parser = sm_singletons.argument_parser
    parser.add_argument('-n', '--new', action='store_true', help=_("whether to create a new state-machine"))
    parser.add_argument('-o', '--open', action='store', nargs='*', type=parse_state_machine_path,
                        dest='state_machine_paths', metavar='path', help=_(
            "specify directories of state-machines that shall be opened. Paths must contain a statemachine.yaml file"))
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help=_(
                            "path to the configuration file config.yaml. Use 'None' to prevent the generation of a config file and use the default configuration. Default: {0}").format(
                            home_path))
    parser.add_argument('-g', '--gui_config', action='store', type=config_path, metavar='path', dest='gui_config_path',
                        default=home_path, nargs='?', const=home_path, help=_(
            "path to the configuration file gui_config.yaml. Use 'None' to prevent the generation of a config file and use the default configuration. Default: {0}").format(
            home_path))
    parser.add_argument('-ss', '--start_state_machine', metavar='path', dest='start_state_machine_flag',
                        default=False, nargs='?',
                        help=_("a flag to specify if the state machine should be started after launching rafcon"))
    parser.add_argument('-s', '--start_state_path', metavar='path', dest='start_state_path',
                        default=None, nargs='?', help=_("path of to the state that should be launched"))
    parser.add_argument('-q', '--quit', metavar='path', dest='quit_flag',
                        default=False, nargs='?',
                        help=_("a flag to specify if the gui should quit after launching a state machine"))
    return parser


def setup_mvc_configuration(core_config_path, gui_config_path, runtime_config_path):
    setup_configuration(core_config_path)
    global_gui_config.load(gui_config_path)
    global_runtime_config.load(runtime_config_path)


def setup_gui():
    # Create the GUI-View
    main_window_view = MainWindowView()
    sm_manager_model = mvc_singletons.state_machine_manager_model
    main_window_controller = MainWindowController(sm_manager_model, main_window_view)
    return main_window_controller


def open_state_machines(paths):
    for path in paths:
        try:
            state_machine = storage.load_state_machine_from_path(path)
            sm_singletons.state_machine_manager.add_state_machine(state_machine)
            return state_machine
        except Exception as e:
            logger.exception(_("Could not load state machine '{}': {}").format(path, e))


def create_new_state_machine():
    root_state = HierarchyState()
    state_machine = StateMachine(root_state)
    sm_singletons.state_machine_manager.add_state_machine(state_machine)


def log_ready_output():
    # Ensure that the next message is being printed (needed for LN manager to detect finished startup)
    level = logger.level
    logger.setLevel(logging.INFO)
    logger.info("Ready")
    logger.setLevel(level)


SIGNALS_TO_NAMES_DICT = dict((getattr(signal, n), n) for n in dir(signal) if n.startswith('SIG') and '_' not in n)


def signal_handler(signal, frame):
    from rafcon.statemachine.enums import StateMachineExecutionStatus
    state_machine_execution_engine = sm_singletons.state_machine_execution_engine
    sm_singletons.shut_down_signal = signal

    try:
        # in this case the print is on purpose the see more easily if the interrupt signal reached the thread
        print _("Signal '{}' received.\nExecution engine will be stopped and program will be shutdown!").format(
            SIGNALS_TO_NAMES_DICT.get(signal, "[unknown]"))
        if state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
            state_machine_execution_engine.stop()
            state_machine_execution_engine.join(3)  # Wait max 3 sec for the execution to stop
    except Exception as e:
        import traceback
        print _("Could not stop state machine: {0} {1}").format(e.message, traceback.format_exc())

    mvc_singletons.main_window_controller.get_controller('menu_bar_controller').prepare_destruction()

    # shutdown twisted correctly
    if reactor_required():
        from twisted.internet import reactor
        if reactor.running:
            reactor.callFromThread(reactor.stop)

    gtk.main_quit()

    plugins.run_hook("post_destruction")


if __name__ == '__main__':
    register_signal_handlers(signal_handler)

    setup_l10n()
    setup_l10n_gtk()

    setup_gtkmvc_logger()
    pre_setup_plugins()

    # from rafcon.utils import log
    logger.info(_("RAFCON launcher"))

    setup_mvc_environment()

    parser = setup_argument_parser()
    user_input = parser.parse_args()

    # create lock file
    if global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
        constants.RAFCON_INSTANCE_LOCK_FILE = open(os.path.join(RAFCON_TEMP_PATH_BASE, 'lock'), 'a+')
        constants.RAFCON_INSTANCE_LOCK_FILE.close()

    setup_mvc_configuration(user_input.config_path, user_input.gui_config_path, user_input.gui_config_path)

    if user_input.state_machine_paths:
        state_machine = open_state_machines(user_input.state_machine_paths)

    if user_input.new:
        create_new_state_machine()

    main_window_controller = setup_gui()

    post_setup_plugins(user_input)

    log_ready_output()

    if global_config.get_config_value("PROFILER_RUN", False):
        profiler.start("global")

    if user_input.start_state_machine_flag:
        start_state_machine(state_machine, user_input.start_state_path, user_input.quit_flag)

    try:
        # check if twisted is imported
        if reactor_required():
            from twisted.internet import reactor

            reactor.run()
        else:
            gtk.main()

        logger.info(_("Main window was closed"))

    finally:
        plugins.run_hook("post_destruction")

        if global_config.get_config_value("PROFILER_RUN", False):
            result_path = global_config.get_config_value("PROFILER_RESULT_PATH")
            view = global_config.get_config_value("PROFILER_VIEWER")
            profiler.stop("global", result_path, view)

        if global_gui_config.get_config_value('AUTO_RECOVERY_LOCK_ENABLED'):
            if os.path.exists(constants.RAFCON_INSTANCE_LOCK_FILE.name):
                os.remove(constants.RAFCON_INSTANCE_LOCK_FILE.name)
            else:
                logger.warning(_("External remove of lock file detected!"))

    if sm_singletons.state_machine_execution_engine.status.execution_mode == StateMachineExecutionStatus.STARTED:
        logger.info(_("Waiting for the state machine execution to finish"))
        sm_singletons.state_machine_execution_engine.join()
        logger.info(_("State machine execution has finished"))

    logger.info(_("Exiting ..."))

    # this is a ugly process shutdown method but works if gtk or twisted process are still blocking
    # import os
    # os._exit(0)
