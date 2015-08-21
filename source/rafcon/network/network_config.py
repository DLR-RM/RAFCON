import os
from rafcon.utils.config import DefaultConfig, ConfigError, read_file
from rafcon.utils import log
logger = log.get_logger(__name__)

CONFIG_FILE = "net_config.yaml"

DEFAULT_CONFIG = read_file(os.path.dirname(__file__), CONFIG_FILE)


class NetworkConfig(DefaultConfig):
    """
    Class to hold and load the global GUI configurations.
    """

    def __init__(self):
        super(NetworkConfig, self).__init__(DEFAULT_CONFIG)
        # self.load(CONFIG_FILE)
        if self.get_config_value("TYPE") != "NETWORK_CONFIG":
            raise ConfigError("Type should be NETWORK_CONFIG for Network configuration. "
                              "Please add \"TYPE: NETWORK_CONFIG\" to your net_config.yaml file.")

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