from awesome_tool.utils.config import DefaultConfig, ConfigError
from awesome_tool.utils import helper
from awesome_tool.utils import log
logger = log.get_logger(__name__)

DEFAULT_CONFIG = """

TYPE: NETWORK_CONFIG

SERVER_IP: 127.0.0.1
SERVER_TCP_PORT: 8888
SERVER_UDP_PORT: 9999

SELF_UDP_PORT: 7777
NUMBER_UDP_MESSAGES_SENT: 10
NUMBER_UDP_MESSAGES_HISTORY: 100

NUMBER_OF_UDP_PACKAGES_UNTIL_TIMEOUT: 15
SECONDS_BETWEEN_UDP_RESEND: 2

SPACEBOT_CUP_MODE: False
AUTOCONNECT_UDP_TO_SERVER: True
AUTOCONNECT_TCP_TO_SERVER: False
"""

CONFIG_FILE = "net_config.yaml"


class NetworkConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    def __init__(self):
        super(NetworkConfig, self).__init__(DEFAULT_CONFIG)
        # sm_path, gui_path, net_path = helper.get_opt_paths()
        # DefaultConfig.__init__(self, CONFIG_FILE, DEFAULT_CONFIG, net_path)
        if self.get_config_value("TYPE") != "NETWORK_CONFIG":
            raise ConfigError("Type should be NETWORK_CONFIG for Network configuration. "
                              "Please add \"TYPE: NETWORK_CONFIG\" to your net_config.yaml file.")

        self.load(CONFIG_FILE)

    def load(self, config_file=None, path=None):
        if config_file is None:
            config_file = CONFIG_FILE
        super(NetworkConfig, self).load(config_file, path)

    def get_server_ip(self):
        return self.get_config_value("SERVER_IP")

    def get_server_udp_port(self):
        return self.get_config_value("SERVER_UDP_PORT")

    def get_server_tcp_port(self):
        return self.get_config_value("SERVER_TCP_PORT")

global_net_config = NetworkConfig()