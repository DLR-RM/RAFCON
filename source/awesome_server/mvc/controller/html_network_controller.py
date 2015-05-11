import os
import weakref
import gobject

from twisted.web import server, resource, static
from twisted.internet import reactor

from awesome_server.utils.config import global_server_config

from awesome_server.utils import log
logger = log.get_logger(__name__)


class HtmlNetworkController(resource.Resource, gobject.GObject):

    def __init__(self):
        self.__gobject_init__()
        self.isLeaf = True
        self.sse_conns = weakref.WeakSet()

        self.path_to_static_files = os.path.dirname(os.path.realpath(__file__)) + "/../../html_files"

    def render_GET(self, request):
        request.setHeader('Content-Type', 'text/event-stream')
        self.sse_conns.add(request)
        return server.NOT_DONE_YET

    def send_data(self, data, ip, port, data_flag):
        for conn in self.sse_conns:
            if data_flag == "REG":
                conn.write("event: registration\n")
            #event_line = #, \"ip\": \"%s\", \"port\": \"%d\"}\n\n" % (data, ip, port)
            conn.write("data: %s\n" % data)
            conn.write("data: %s\n" % ip)
            conn.write("data: %d\n\n" % port)

    def start_html_server(self):
        port = global_server_config.get_config_value("HTML_SERVER_PORT")
        logger.debug("Starting HTML server at port %d" % port)

        root = resource.Resource()
        root.putChild('', DebugPage(self))
        root.putChild('debug', DebugPage(self))
        root.putChild('js', static.File(self.path_to_static_files + "/js"))
        root.putChild('my_event_source', self)

        reactor.listenTCP(port, server.Site(root))


class DebugPage(resource.Resource):

    def __init__(self, controller):
        self.path_to_static_files = os.path.dirname(os.path.realpath(__file__)) + "/../../html_files"

        self.controller = controller

    def render_GET(self, request):
        f = open(os.path.join(self.path_to_static_files, "index.html"))
        index = f.read()
        f.close()
        return index

    def render_POST(self, request):
        command = request.args["command"][0]
        ip, port = request.args["addr"][0].split(":")
        if command and ip and port:
            logger.debug("Received command: \'%s\'" % command)
            self.controller.emit("command_received", command, ip, int(port))
        return "Success"


gobject.type_register(HtmlNetworkController)
gobject.signal_new("command_received", HtmlNetworkController, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_STRING,
                                                                                               gobject.TYPE_STRING,
                                                                                               gobject.TYPE_INT))