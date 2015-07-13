from awesome_tool.network.network_connections import NetworkConnections
from awesome_tool.network.network_controller import TCPNetworkController, UDPNetworkController

udp_net_controller = UDPNetworkController()
tcp_net_controller = TCPNetworkController()

network_connections = NetworkConnections(udp_net_controller, tcp_net_controller)