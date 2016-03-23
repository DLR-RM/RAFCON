"""
.. module:: start client
   :platform: Unix, Windows
   :synopsis: A module to start a unit test RAFCON client instance

.. moduleauthor:: Sebastian Brunner


"""

import logging
import gtk
import signal
from os.path import realpath, dirname, join


def setup_logger():
    import sys
    # Set the views for the loggers

    # Apply defaults to logger of gtkmvc
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)


def start_client(interacting_function, queue_dict):

    import rafcon
    from rafcon.utils.config import config_path
    from rafcon.utils import log
    from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
    import rafcon.utils.filesystem as filesystem

    from rafcon.statemachine.config import global_config
    from rafcon.statemachine.storage.storage import StateMachineStorage
    from rafcon.statemachine.state_machine import StateMachine
    from rafcon.statemachine.states.hierarchy_state import HierarchyState
    import rafcon.statemachine.singleton as sm_singletons

    from rafcon.mvc.controllers.main_window import MainWindowController
    from rafcon.mvc.views.main_window import MainWindowView
    import rafcon.mvc.singleton as mvc_singletons
    from rafcon.mvc.config import global_gui_config
    from rafcon.mvc.runtime_config import global_runtime_config

    from plugins import monitoring

    try:
        from plugins.monitoring.monitoring_manager import global_monitoring_manager
        from twisted.internet import gtk2reactor
        # needed for glib.idle_add, and signals
        gtk2reactor.install()
        from twisted.internet import reactor
    except ImportError, e:
        print "Monitoring plugin not found"

    setup_logger()
    logger = log.get_logger("start")
    logger.info("RAFCON launcher")

    rafcon_root_path = dirname(realpath(rafcon.__file__))
    import os
    if not os.environ.get('RAFCON_PATH', None):
        # set env variable RAFCON_PATH to the root directory of RAFCON
        os.environ['RAFCON_PATH'] = rafcon_root_path

    if not os.environ.get('RAFCON_LIB_PATH', None):
        # set env variable RAFCON_LIB_PATH to the library directory of RAFCON (when not using RMPM)
        os.environ['RAFCON_LIB_PATH'] = join(dirname(rafcon_root_path), 'libraries')

    signal.signal(signal.SIGINT, sm_singletons.signal_handler)

    global_config.load(path=os.path.dirname(os.path.abspath(__file__)))
    global_gui_config.load(path=os.path.dirname(os.path.abspath(__file__)))
    global_runtime_config.load(path=os.path.dirname(os.path.abspath(__file__)))

    setup_config = dict()
    setup_config["net_config_path"] = os.path.abspath(path=os.path.dirname(os.path.abspath(__file__))+"/client")

    # Make mvc directory the working directory
    # Needed for views, which assume to be in the mvc path and import glade files relatively
    os.chdir(join(rafcon_root_path, 'mvc'))

    # Initialize library
    sm_singletons.library_manager.initialize()

    # Set base path of global storage
    sm_singletons.global_storage.base_path = RAFCON_TEMP_PATH_STORAGE

    # Create the GUI
    main_window_view = MainWindowView()

    storage = StateMachineStorage()
    state_machine, version, creation_time = storage.load_statemachine_from_path(
        "../../test_scripts/unit_test_state_machines/99_bottles_of_beer_monitoring")
    sm_singletons.state_machine_manager.add_state_machine(state_machine)

    sm_manager_model = mvc_singletons.state_machine_manager_model
    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type='LogicDataGrouped')

    try:
        # check if monitoring plugin is loaded
        from plugins.monitoring.monitoring_manager import global_monitoring_manager

        def initialize_monitoring_manager():
            monitoring_manager_initialized = False
            while not monitoring_manager_initialized:
                logger.info("Try to initialize the global monitoring manager and setup the connection to the server!")
                succeeded = global_monitoring_manager.initialize(setup_config)
                if succeeded:
                    monitoring_manager_initialized = True

        import threading
        init_thread = threading.Thread(target=initialize_monitoring_manager)
        init_thread.start()

        interacting_thread = threading.Thread(target=interacting_function, args=[main_window_controller,
                                                                                 global_monitoring_manager,
                                                                                 queue_dict])
        interacting_thread.start()

        if global_monitoring_manager.networking_enabled():
            # gtk.main()
            reactor.run()
        else:
            gtk.main()

    except ImportError, e:
        logger.info("Monitoring plugin not found: executing the GUI directly")
        # plugin not found
        gtk.main()

    logger.info("Joined root state")

    # If there is a running state-machine, wait for it to be finished before exiting
    sm = sm_singletons.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()

    logger.info("Exiting ...")

    # this is a ugly process shutdown method but works if gtk or twisted process are still blocking
    import os
    os._exit(0)


def print_objects(main_window_controller, global_monitoring_manager, queue_dict):
    print "dummy prints:"
    print main_window_controller
    print global_monitoring_manager
    print queue_dict

if __name__ == '__main__':
    start_client(print_objects, "multiprocessing_queue_dict")
