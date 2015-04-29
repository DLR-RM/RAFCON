from twisted.internet.protocol import DatagramProtocol

from gtkmvc import Observable


class UDPConnection(DatagramProtocol, Observable):

    def __init__(self, view):
        Observable.__init__(self)

        self.view = view
        self.clients = []

    def datagramReceived(self, data, addr):
        if addr not in self.clients:
            self.append_client(addr)

        self.print_message("SERVER: %s from %s\n" % (data, repr(addr)))
        self.transport.write(data, addr)

    @Observable.observed
    def append_client(self, addr):
        self.clients.append(addr)
        self.print_message("SERVER: New connection to %s\n" % repr(addr))

    def send_message(self, message, addr):
        for i in range(0, 10):
            self.transport.write(message + "#%d" % i, addr)

    def print_message(self, message):
        buffer = self.view["textview"].get_buffer()
        buffer.insert(buffer.get_end_iter(), message)