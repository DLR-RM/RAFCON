# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

# TODO: will be replaced soon
# from rafcon.gui.controllers.extended_controller import ExtendedController
# from rafcon.gui.models.state_machine_manager import StateMachineManagerModel
# from rafcon.utils import log
# from rafcon.gui.views.network_connections import NetworkConnectionsView
# from rafcon.acknowledged_udp.singleton import network_connections
#
# logger = log.get_logger(__name__)
#
#
# class NetworkController(ExtendedController):
#     def __init__(self, model, view):
#         assert isinstance(model, StateMachineManagerModel)
#         assert isinstance(view, NetworkConnectionsView)
#         ExtendedController.__init__(self, model, view)
#
#         self._network_connections = network_connections
#
#         self._network_connections.connect('tcp_connected', self.tcp_connected)
#         self._network_connections.connect('tcp_disconnected', self.tcp_disconnected)
#         self._network_connections.connect('udp_response_received', self.udp_response_received)
#         self._network_connections.connect('udp_no_response_received', self.udp_no_response_received)
#
#     @property
#     def network_connections(self):
#         return self._network_connections
#
#     def tcp_connected(self, nt_con):
#         self.view["tcp_connection_connected_label"].set_text("YES")
#         self.view["tcp_connect_button"].set_label("Disconnect TCP")
#
#     def tcp_disconnected(self, nt_con):
#         self.view["tcp_connection_connected_label"].set_text("NO")
#         self.view["tcp_connect_button"].set_label("Connect TCP")
#
#     def udp_response_received(self, nt_con):
#         self.view["udp_connection_registered_label"].set_text("YES")
#
#     def udp_no_response_received(self, nt_con):
#         self.view["udp_connection_registered_label"].set_text("NO")
#
#     def on_udp_register_button_clicked(self, widget, event=None):
#         if global_net_config.get_config_value("SPACEBOT_CUP_MODE"):
#             self.view["udp_connection_registered_label"].set_text("YES")
#         self.network_connections.register_udp()
#
#     def on_tcp_connect_button_clicked(self, widget, event=None):
#         self.network_connections.connect_tcp()
