import os
import weakref
import gobject

from twisted.web import server, resource, static
from twisted.internet import reactor

from rafcon.network.network_config import global_net_config
from rafcon.utils import constants

from rafcon.utils import log
logger = log.get_logger(__name__)


class HtmlNetworkController(resource.Resource, gobject.GObject):

    def __init__(self, udp_net_controller):
        super(HtmlNetworkController, self).__init__()
        self.__gobject_init__()
        self.isLeaf = True
        self.sse_conns = weakref.WeakSet()
        self.path_to_static_files = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                 "html_files")
        self.udp_net_controller = udp_net_controller

    def render_GET(self, request):
        request.setHeader('Content-Type', 'text/event-stream')
        self.sse_conns.add(request)
        return server.NOT_DONE_YET

    def send_data(self, data, ip, port, data_flag, sm_name):
        for conn in self.sse_conns:
            self.send_data_to_request(conn, data, ip, port, data_flag, sm_name)

    def send_data_to_request(self, request, data, ip, port, data_flag, sm_name):
        json_pkg = ""
        if data_flag == "REG":
            json_pkg = "event: registration\n" \
                       "id: 1\n"
        elif data_flag == "EXE":
            json_pkg = "event: execution\n" \
                       "id: 2\n"
        elif data_flag == "ASC":
            json_pkg = "event: active_state_changed\n" \
                       "id: 3\n"

        json_pkg += "data: {" \
                   "\"msg\": \"%s\", " \
                   "\"sm_name\": \"%s\", " \
                   "\"ip\": \"%s\", " \
                   "\"port\": \"%d\"}\n\n" % (data, str(sm_name), ip, port)

        request.write(json_pkg)

    def send_state_data(self, sm_name, parent_id, state_id, state_name, pos_x, pos_y, width, height, hierarchy_level,
                        outcomes):
        json_pkg = "event: new_state\n" \
                   "id: 4\n" \
                   "data: {" \
                   "\"sm_name\": \"%s\"," \
                   "\"parent_id\": \"%s\"," \
                   "\"state_id\": \"%s\"," \
                   "\"state_name\": \"%s\"," \
                   "\"pos_x\": \"%f\"," \
                   "\"pos_y\": \"%f\"," \
                   "\"width\": \"%f\"," \
                   "\"height\": \"%f\"," \
                   "\"hierarchy_level\": \"%d\"," \
                   "\"outcomes\": [" % (str(sm_name),
                                        str(parent_id),
                                        str(state_id),
                                        str(state_name),
                                        pos_x,
                                        pos_y,
                                        width,
                                        height,
                                        hierarchy_level)

        for outcome_id, outcome in outcomes.iteritems():
            if outcome_id >= 0:
                json_pkg += "\"%s\"," % str(outcome.name)

        if len(outcomes) > 2:
            json_pkg = "]}\n\n".join(json_pkg.rsplit(",", 1))
        else:
            json_pkg += "]}\n\n"

        for conn in self.sse_conns:
            conn.write(json_pkg)

    def send_connection_data(self, sm_name, container_state_id, from_outcome, from_state, to_outcome, to_state, waypoints):
        json_pkg = "event: new_connection\n" \
                   "id: 5\n" \
                   "data: {" \
                   "\"sm_name\": \"%s\"," \
                   "\"container_state_id\": \"%s\"," \
                   "\"from_outcome\": \"%s\"," \
                   "\"from_state\": \"%s\"," \
                   "\"to_outcome\": \"%s\"," \
                   "\"to_state\": \"%s\"," \
                   "\"waypoints\": [" % (str(sm_name),
                                         str(container_state_id),
                                         str(from_outcome),
                                         str(from_state),
                                         str(to_outcome),
                                         str(to_state))

        if waypoints:
            for point in waypoints:
                json_pkg += "[%f, %f], " % (abs(point[0] * constants.BROWSER_SIZE_MULTIPLIER),
                                            abs(point[1] * constants.BROWSER_SIZE_MULTIPLIER))
            json_pkg = "]}\n\n".join(json_pkg.rsplit(",", 1))
        else:
            json_pkg += "]}\n\n"

        for conn in self.sse_conns:
            conn.write(json_pkg)

    def send_sm_transmission_end(self):
        json_pkg = "event: sm_transmission_end\n" \
                   "data: none\n\n"

        for conn in self.sse_conns:
            conn.write(json_pkg)

    def start_html_server(self):
        port = global_net_config.get_config_value("HTML_SERVER_PORT", 8889)
        logger.debug("Starting HTML server at port %d" % port)

        root = resource.Resource()
        root.putChild('', IndexPage(self))
        root.putChild('debug', DebugPage(self))
        root.putChild('js', static.File(self.path_to_static_files + "/js"))
        root.putChild('css', static.File(self.path_to_static_files + "/css"))
        root.putChild('jointjs_test', JointJSTestPage(self))
        root.putChild('state_machine_event_source', self)

        reactor.listenTCP(port, server.Site(root))


class DefaultPage(resource.Resource):

    def __init__(self, controller, page_name):
        self.path_to_static_files = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                 "html_files")

        self.controller = controller
        self.page_name = page_name

    def render_GET(self, request):
        f = open(os.path.join(self.path_to_static_files, self.page_name))
        page = f.read()
        f.close()

        selector = ""

        for conn in self.controller.udp_net_controller.get_connections().itervalues():
            for client_name, client_addr in conn.clients.iteritems():
                selector += "<option value=\"%s:%d\">%s</option>\n" % (client_addr[0], client_addr[1], client_name)

        if selector == "":
            return page
        else:
            page = page.replace("<option value=\"none\">No statemachine.</option>", selector)
            return page

    def render_POST(self, request):
        command = request.args["command"][0]
        sm_name = request.args["sm_name"][0]
        ip, port = request.args["addr"][0].split(":")
        if command and ip and port:
            logger.debug("Received command: \'%s\'" % command)
            self.controller.emit("command_received", command, sm_name, ip, int(port))
        return "Success"


class JointJSTestPage(DefaultPage):

    def __init__(self, controller):
        DefaultPage.__init__(self, controller, "jointjs_test.html")


class IndexPage(DefaultPage):

    def __init__(self, controller):
        DefaultPage.__init__(self, controller, "index.html")


class DebugPage(DefaultPage):

    def __init__(self, controller):
        DefaultPage.__init__(self, controller, "debug.html")


gobject.type_register(HtmlNetworkController)
gobject.signal_new("command_received", HtmlNetworkController, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_STRING,
                                                                                               gobject.TYPE_STRING,
                                                                                               gobject.TYPE_STRING,
                                                                                               gobject.TYPE_INT))