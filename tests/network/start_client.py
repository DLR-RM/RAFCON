"""
.. module:: start client
   :platform: Unix, Windows
   :synopsis: A module to start a unit test RAFCON client instance

.. moduleauthor:: Sebastian Brunner


"""

import logging
import gtk
import signal
import sys
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
    from rafcon.gui.config import global_gui_config
    import os
    
    from rafcon.gui.controllers.main_window import MainWindowController
    from rafcon.gui.views.main_window import MainWindowView
    import rafcon.gui.singleton as gui_singletons
    from rafcon.gui.runtime_config import global_runtime_config
    from rafcon.gui.start import signal_handler

    import rafcon
    from rafcon.utils import log
    from rafcon.utils import plugins

    from rafcon.core.config import global_config
    from rafcon.core.storage import storage as global_storage
    from rafcon.core.state_machine import StateMachine
    from rafcon.core.states.hierarchy_state import HierarchyState
    import rafcon.core.singleton as core_singletons
    from rafcon.core.start import setup_environment

    # load all plugins specified in the RAFCON_PLUGIN_PATH
    plugins.load_plugins()
    import testing_utils

    # check if twisted is imported
    if "twisted" in sys.modules.keys():
        from twisted.internet import gtk2reactor
        # needed for glib.idle_add, and signals
        gtk2reactor.install()
        from twisted.internet import reactor
    else:
        print "Twisted not imported! Thus the gkt2reatcor is not installed!"
        exit()

    plugins.run_pre_inits()

    setup_logger()
    logger = log.get_logger("start")
    logger.info("RAFCON launcher")

    setup_environment()

    signal.signal(signal.SIGINT, signal_handler)

    global_config.load(path=os.path.dirname(os.path.abspath(__file__)))
    global_gui_config.load(path=os.path.dirname(os.path.abspath(__file__)))
    global_runtime_config.load(path=os.path.dirname(os.path.abspath(__file__)))

    setup_config = dict()
    setup_config["net_config_path"] = os.path.abspath(path=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                        "client"))

    # Initialize library
    core_singletons.library_manager.initialize()

    # Create the GUI
    main_window_view = MainWindowView()

    state_machine = global_storage.load_state_machine_from_path(
        testing_utils.get_test_sm_path(os.path.join("unit_test_state_machines", "99_bottles_of_beer_monitoring")))

    sm_id = rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = sm_id

    sm_manager_model = gui_singletons.state_machine_manager_model
    main_window_controller = MainWindowController(sm_manager_model, main_window_view)

    plugins.run_post_inits(setup_config)

    import threading
    # this is not recognized by pycharm as the module is loaded in plugins.load_plugins()
    from monitoring.monitoring_manager import global_monitoring_manager
    interacting_thread = threading.Thread(target=interacting_function, args=[main_window_controller,
                                                                             global_monitoring_manager,
                                                                             queue_dict])
    testing_utils.wait_for_gui()
    interacting_thread.start()

    # check if twisted is imported
    if "twisted" in sys.modules.keys():
        reactor.run()
    else:
        logger.error("Client: Twisted is not in sys.modules or twisted is not working! Exiting program ... !")
        os._exit(0)

    logger.info("Joined root state")

    # If there is a running state-machine, wait for it to be finished before exiting
    sm = core_singletons.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()

    logger.info("Exiting ...")

    # this is a ugly process shutdown method but works if gtk or twisted process are still blocking
    os._exit(0)


def print_objects(main_window_controller, global_monitoring_manager, queue_dict):
    print "dummy prints:"
    print main_window_controller
    print global_monitoring_manager
    print queue_dict

if __name__ == '__main__':
    start_client(print_objects, "multiprocessing_queue_dict")
