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

SECONDS_FOR_UDP_TIMEOUT: 60
SECONDS_BETWEEN_UDP_RESEND: 5

SPACEBOARD_CUP_MODE: False
"""

CONFIG_FILE = "net_config.yaml"


class NetworkConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    def __init__(self):
        sm_path, gui_path, net_path = helper.get_opt_paths()
        DefaultConfig.__init__(self, CONFIG_FILE, DEFAULT_CONFIG, net_path)
        if self.get_config_value("TYPE") != "NETWORK_CONFIG":
            raise ConfigError("Type should be NETWORK_CONFIG for Network configuration. "
                              "Please add \"TYPE: NETWORK_CONFIG\" to your net_config.yaml file.")

global_net_config = NetworkConfig()