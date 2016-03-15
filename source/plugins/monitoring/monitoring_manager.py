"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the state machine

.. moduleauthor:: Sebastian Brunner


"""

from plugins.monitoring.client import MonitoringClient
from plugins.monitoring.server import MonitoringServer
from network.config import global_network_config

from gtkmvc.slim_observer import SlimObserver

from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.container_state import ContainerState
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.singleton import state_machine_manager
from rafcon.statemachine.singleton import argument_parser

from rafcon.utils.config import config_path
import rafcon.utils.filesystem as filesystem

from rafcon.utils import log
logger = log.get_logger(__name__)


class MonitoringManager(SlimObserver):

    def __init__(self):

        home_path = filesystem.get_home_path()

        argument_parser.add_argument(
            '-nc', '--net_config', action='store', type=config_path, metavar='path', dest='net_config_path',
            default=home_path, nargs='?', const=home_path,
            help="path to the configuration file net_config.yaml. Use 'None' to prevent the generation of "
                 "a config file and use the default configuration. Default: {0}".format(home_path))

        self.endpoint = None

    def initialize(self, setup_config):

        global_network_config.load(path=setup_config['net_config_path'])

        if global_network_config.get_config_value("SERVER", True):
            self.endpoint = MonitoringServer()
            self.endpoint.connect()
        else:
            self.endpoint = MonitoringClient()
            self.endpoint.connect()

    def shutdown(self):
        if self.endpoint:
            self.endpoint.shutdown()

    @staticmethod
    def networking_enabled():
        return global_network_config.get_config_value("ENABLED", False)


global_monitoring_manager = MonitoringManager()