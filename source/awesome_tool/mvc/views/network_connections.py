from gtkmvc import View


class NetworkConnectionsView(View):
    builder = './glade/network_connections_view.glade'
    top = 'network_connections_view'

    def __init__(self):
        View.__init__(self)