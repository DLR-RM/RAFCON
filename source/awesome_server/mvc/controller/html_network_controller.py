import os
import weakref
import gobject

from twisted.web import server, resource, static
from twisted.internet import reactor

from awesome_server.utils.config import global_server_config
from awesome_server.utils import constants

from awesome_server.utils import log
logger = log.get_logger(__name__)


class HtmlNetworkController(resource.Resource, gobject.GObject):

    def __init__(self, connection_manager):
        self.__gobject_init__()
        self.isLeaf = True
        self.sse_conns = weakref.WeakSet()

        self.path_to_static_files = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                 os.pardir,
                                                 os.pardir,
                                                 "html_files")

        self.connection_manager = connection_manager

    def render_GET(self, request):
        request.setHeader('Content-Type', 'text/event-stream')
        self.sse_conns.add(request)
        return server.NOT_DONE_YET

    def send_data(self, data, ip, port, data_flag):
        for conn in self.sse_conns:
            self.send_data_to_request(conn, data, ip, port, data_flag)

    def send_data_to_request(self, request, data, ip, port, data_flag):
        if data_flag == "REG":
            request.write("event: registration\n")
        elif data_flag == "EXE":
            request.write("event: execution\n")

        request.write("data: %s\n" % data)
        request.write("data: %s\n" % ip)
        request.write("data: %d\n\n" % port)

    def send_state_data(self, parent_id, state_id, state_name, pos_x, pos_y, width, height, hierarchy_level, outcomes):
        for conn in self.sse_conns:
            conn.write("event: new_state\n")

            conn.write("data: %s\n" % parent_id)
            conn.write("data: %s\n" % state_id)
            conn.write("data: %s\n" % state_name)
            conn.write("data: %d\n" % pos_x)
            conn.write("data: %d\n" % pos_y)
            conn.write("data: %d\n" % width)
            conn.write("data: %d\n" % height)
            conn.write("data: %d\n" % hierarchy_level)
            for outcome_id, outcome in outcomes.iteritems():
                if outcome_id >= 0:
                    conn.write("data: %s\n" % outcome.name)
            conn.write("\n")

    def send_connection_data(self, container_state_id, from_outcome, from_state, to_outcome, to_state, waypoints):
        for conn in self.sse_conns:
            conn.write("event: new_connection\n")

            conn.write("data: %s\n" % container_state_id)
            conn.write("data: %s\n" % from_outcome)
            conn.write("data: %s\n" % from_state)
            conn.write("data: %s\n" % to_outcome)
            conn.write("data: %s\n" % to_state)
            if waypoints:
                for point in waypoints:
                    conn.write("data: %f\n" % abs(point[0] * constants.BROWSER_SIZE_MULTIPLIER))
                    conn.write("data: %f\n" % abs(point[1] * constants.BROWSER_SIZE_MULTIPLIER))
            conn.write("\n")

    def start_html_server(self):
        port = global_server_config.get_config_value("HTML_SERVER_PORT")
        logger.debug("Starting HTML server at port %d" % port)

        root = resource.Resource()
        root.putChild('', IndexPage(self))
        root.putChild('debug', DebugPage(self))
        root.putChild('js', static.File(self.path_to_static_files + "/js"))
        root.putChild('css', static.File(self.path_to_static_files + "/css"))
        root.putChild('my_event_source', self)

        reactor.listenTCP(port, server.Site(root))


class DefaultPage(resource.Resource):

    def __init__(self, controller, page_name):
        self.path_to_static_files = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                 os.pardir,
                                                 os.pardir,
                                                 "html_files")

        self.controller = controller
        self.page_name = page_name

    def render_GET(self, request):
        f = open(os.path.join(self.path_to_static_files, self.page_name))
        page = f.read()
        f.close()

        selector = ""

        for conn in self.controller.connection_manager.udp_connections:
            for client_name, client_addr in conn.clients.iteritems():
                selector += "<option value=\"%s:%d\">%s</option>\n" % (client_addr[0], client_addr[1], client_name)

        if selector == "":
            return page
        else:
            page = page.replace("<option value=\"none\">No statemachine.</option>", selector)
            return page

    def render_POST(self, request):
        command = request.args["command"][0]
        ip, port = request.args["addr"][0].split(":")
        if command and ip and port:
            logger.debug("Received command: \'%s\'" % command)
            self.controller.emit("command_received", command, ip, int(port))
        return "Success"


class IndexPage(DefaultPage):

    def __init__(self, controller):
        DefaultPage.__init__(self, controller, "index.html")


class DebugPage(DefaultPage):

    def __init__(self, controller):
        DefaultPage.__init__(self, controller, "debug.html")


gobject.type_register(HtmlNetworkController)
gobject.signal_new("command_received", HtmlNetworkController, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_STRING,
                                                                                               gobject.TYPE_STRING,
                                                                                               gobject.TYPE_INT))