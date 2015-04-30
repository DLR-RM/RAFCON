from twisted.internet.protocol import DatagramProtocol

from gtkmvc import Observable

import gobject
from awesome_server.utils import messaging


class UDPConnection(DatagramProtocol, Observable, gobject.GObject):

    def __init__(self):
        self.__gobject_init__()
        Observable.__init__(self)

        self.clients = []

    def datagramReceived(self, data, addr):
        if addr not in self.clients:
            self.append_client(addr)

        ip, port = addr
        self.emit("data_received", data, ip, port)

    @Observable.observed
    def append_client(self, addr):
        self.clients.append(addr)

    def send_message(self, message, addr):
        encr_msg = messaging.create_send_message(message)
        for i in range(0, 10):
            self.transport.write(encr_msg, addr)

    def send_acknowledge(self, message, addr):
        self.send_message(message + "ACK", addr)


gobject.type_register(UDPConnection)
gobject.signal_new("data_received", UDPConnection, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_STRING,
                                                                                    gobject.TYPE_STRING,
                                                                                    gobject.TYPE_INT))