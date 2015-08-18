from twisted.internet import gtk2reactor
gtk2reactor.install()

from twisted.internet import reactor

from rafcon_server.mvc.views.debug_view import DebugView
from rafcon_server.mvc.controller.debug_view import DebugViewController

from rafcon_server.mvc.controller.connection_manager import ConnectionManager
from rafcon_server.mvc.models.connection_manager import ConnectionManagerModel

from rafcon.statemachine.singleton import library_manager

from rafcon.utils import log

import gtk


if __name__ == '__main__':

    library_manager.initialize()

    debug_view = DebugView()

    log.debug_filter.set_logging_test_view(debug_view)
    log.error_filter.set_logging_test_view(debug_view)

    connection_manager = ConnectionManager()
    connection_manager_model = ConnectionManagerModel(connection_manager)

    debug_view_ctrl = DebugViewController(connection_manager_model, debug_view)

    reactor.run()
    gtk.main()