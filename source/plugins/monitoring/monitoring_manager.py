"""
.. module:: monitoring manager
   :platform: Unix, Windows
   :synopsis: A module to hold all execution monitoring functionality

.. moduleauthor:: Sebastian Brunner


"""

from plugins.monitoring.client import MonitoringClient
from plugins.monitoring.server import MonitoringServer
from network.config import global_network_config

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


class MonitoringManager:
    """
    This class holds all monitoring relevant objects. It is configured via a config via given on startup or loaded
    from the default RAFCON config location.
    """

    def __init__(self):

        home_path = filesystem.get_home_path()

        argument_parser.add_argument(
            '-nc', '--net_config', action='store', type=config_path, metavar='path', dest='net_config_path',
            default=home_path, nargs='?', const=home_path,
            help="path to the configuration file net_config.yaml. Use 'None' to prevent the generation of "
                 "a config file and use the default configuration. Default: {0}".format(home_path))

        self.endpoint = None

    def initialize(self, setup_config):
        """
        The is an initialization function, which is called when the connection finally is set up.
        :param setup_config: the setup configuration for the networking
        :return:
        """
        global_network_config.load(path=setup_config['net_config_path'])

        if global_network_config.get_config_value("SERVER", True):
            if not self.endpoint:
                self.endpoint = MonitoringServer()
            return self.endpoint.connect()
        else:
            if not self.endpoint:
                self.endpoint = MonitoringClient()
            return self.endpoint.connect()

    def shutdown(self):
        """
        A function to shutdown the communication endpoint
        :return:
        """
        if self.endpoint:
            self.endpoint.shutdown()

    @staticmethod
    def networking_enabled():
        """
        A method to check if the monitoring capability is globally enabled.
        :return:
        """
        return global_network_config.get_config_value("ENABLED", False)

# this variable is always created when the this module is imported, this is our common way to integrate plugins
global_monitoring_manager = MonitoringManager()