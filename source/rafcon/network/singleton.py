from rafcon.network.network_connections import NetworkConnections
from rafcon.network.network_controller import TCPNetworkController, UDPNetworkController
from rafcon.network.html_network_controller import HtmlNetworkController

udp_net_controller = UDPNetworkController()
tcp_net_controller = TCPNetworkController()

network_connections = NetworkConnections(udp_net_controller, tcp_net_controller)
html_network_controller = HtmlNetworkController(udp_net_controller)