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

    import rafcon
    from yaml_configuration.config import config_path
    from rafcon.utils import log
    from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE
    import rafcon.utils.filesystem as filesystem

    from rafcon.statemachine.config import global_config
    from rafcon.statemachine.storage import storage as global_storage
    from rafcon.statemachine.state_machine import StateMachine
    from rafcon.statemachine.states.hierarchy_state import HierarchyState
    import rafcon.statemachine.singleton as sm_singletons

    from rafcon.mvc.controllers.main_window import MainWindowController
    from rafcon.mvc.views.main_window import MainWindowView
    import rafcon.mvc.singleton as mvc_singletons
    from rafcon.mvc.config import global_gui_config
    from rafcon.mvc.runtime_config import global_runtime_config

    from rafcon.utils import plugins
    # load all plugins specified in the RAFCON_PLUGIN_PATH
    plugins.load_plugins()

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

    rafcon_root_path = dirname(realpath(rafcon.__file__))
    import os
    if not os.environ.get('RAFCON_PATH', None):
        # set env variable RAFCON_PATH to the root directory of RAFCON
        os.environ['RAFCON_PATH'] = rafcon_root_path

    if not os.environ.get('RAFCON_LIB_PATH', None):
        # set env variable RAFCON_LIB_PATH to the library directory of RAFCON (when not using RMPM)
        os.environ['RAFCON_LIB_PATH'] = join(dirname(rafcon_root_path), 'libraries')

    from rafcon.mvc.start import signal_handler
    signal.signal(signal.SIGINT, signal_handler)

    global_config.load(path=os.path.dirname(os.path.abspath(__file__)))
    global_gui_config.load(path=os.path.dirname(os.path.abspath(__file__)))
    global_runtime_config.load(path=os.path.dirname(os.path.abspath(__file__)))

    setup_config = dict()
    setup_config["net_config_path"] = os.path.abspath(path=os.path.dirname(os.path.abspath(__file__))+"/client")

    # Initialize library
    sm_singletons.library_manager.initialize()

    # Create the GUI
    main_window_view = MainWindowView()

    state_machine = global_storage.load_state_machine_from_path(
            rafcon.__path__[0] + "/../test_scripts/unit_test_state_machines/99_bottles_of_beer_monitoring")

    sm_singletons.state_machine_manager.add_state_machine(state_machine)

    sm_manager_model = mvc_singletons.state_machine_manager_model
    main_window_controller = MainWindowController(sm_manager_model, main_window_view, editor_type='LogicDataGrouped')

    plugins.run_post_inits(setup_config)

    import threading
    # this is not recognized by pycharm as the module is loaded in plugins.load_plugins()
    from monitoring.monitoring_manager import global_monitoring_manager
    interacting_thread = threading.Thread(target=interacting_function, args=[main_window_controller,
                                                                             global_monitoring_manager,
                                                                             queue_dict])
    interacting_thread.start()

    # check if twisted is imported
    if "twisted" in sys.modules.keys():
        reactor.run()
    else:
        logger.error("Something went seriously wrong!")
        import os
        os._exit(0)

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
