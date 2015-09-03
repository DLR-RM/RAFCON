from rafcon.utils import log
logger = log.get_logger("rafcon_server")
logger.info("initialize RAFCON ... ")

from twisted.internet import gtk2reactor
gtk2reactor.install()
from twisted.internet import reactor

import gtk
import argparse
from os.path import realpath, dirname, join, exists, expanduser, expandvars, isdir

from rafcon_server.mvc.views.debug_view import DebugView
from rafcon_server.mvc.controller.debug_view import DebugViewController
from rafcon_server.mvc.controller.connection_manager import ConnectionManager
from rafcon_server.mvc.models.connection_manager import ConnectionManagerModel

from rafcon.statemachine.config import global_config
from rafcon.statemachine.singleton import library_manager
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.singleton import global_storage, state_machine_manager

from rafcon.utils.config import config_path

from rafcon.network.network_config import global_net_config


def state_machine_path(path):
    sm_root_file = join(path, StateMachineStorage.STATEMACHINE_FILE)
    if exists(sm_root_file):
        return path
    else:
        raise argparse.ArgumentTypeError("Failed to open {0}: {1} not found in path".format(path,
                                                                                StateMachineStorage.STATEMACHINE_FILE))


if __name__ == '__main__':

    home_path = expanduser('~')
    if home_path:
        home_path = join(home_path, ".config", "rafcon")
    else:
        home_path = 'None'

    logger.info("parse arguments ... ")
    parser = argparse.ArgumentParser(description='Start RAFCON')
    parser.add_argument('-o', '--open', action='store', type=state_machine_path, dest='sm_path', metavar='path',
                        help="specify a directory of a state-machine that shall be opened and started. The path must contain a "
                             "statemachine.yaml file")
    parser.add_argument('-c', '--config', action='store', type=config_path, metavar='path', dest='config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))
    parser.add_argument('-nc', '--net_config', action='store', type=config_path, metavar='path', dest='net_config_path',
                        default=home_path, nargs='?', const=home_path,
                        help="path to the configuration file net_config.yaml. Use 'None' to prevent the generation of "
                             "a config file and use the default configuration. Default: {0}".format(home_path))

    result = parser.parse_args()
    setup_config = vars(result)

    if not setup_config['sm_path']:
        logger.warn("No statemachine specified")

    global_config.load(path=setup_config['config_path'])
    global_net_config.load(path=setup_config['net_config_path'])

    # initialize the logging view
    debug_view = DebugView()
    log.debug_filter.set_logging_test_view(debug_view)
    log.error_filter.set_logging_test_view(debug_view)

    library_manager.initialize()

    [state_machine, version, creation_time] = global_storage.load_statemachine_from_yaml(setup_config['sm_path'])
    state_machine_manager.add_state_machine(state_machine)

    logger.debug("The following statemachine was loaded: {0}".
                 format(state_machine_manager.get_active_state_machine().file_system_path))

    connection_manager = ConnectionManager()
    connection_manager_model = ConnectionManagerModel(connection_manager)

    debug_view_ctrl = DebugViewController(connection_manager_model, debug_view)

    reactor.run()
    #gtk.main()