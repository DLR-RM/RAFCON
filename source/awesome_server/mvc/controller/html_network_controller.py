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

        self.path_to_static_files = os.path.dirname(os.path.realpath(__file__)) + "/html_files/static"

    def render_GET(self, request):
        ip = request.getClientIP()
        print "SSE GET: " + ip
        request.setHeader('Content-Type', 'text/event-stream')
        self.sse_conns.add(request)
        return server.NOT_DONE_YET

    def send_data(self, data):
        for conn in self.sse_conns:
            event_line = "data: {}\n\n".format(data)
            conn.write(event_line)

    def start_html_server(self):
        port = global_server_config.get_config_value("HTML_SERVER_PORT")
        logger.debug("Starting HTML server at port %d" % port)

        root = resource.Resource()
        # root.putChild('', static.File(os.path.join(self.path_to_static_files, "index.html")))
        root.putChild('', FirstPage(self))
        root.putChild('js', static.File(self.path_to_static_files + "/js"))
        root.putChild('my_event_source', self)

        reactor.listenTCP(port, server.Site(root))


class FirstPage(resource.Resource):

    def __init__(self, controller):
        self.path_to_static_files = os.path.dirname(os.path.realpath(__file__)) + "/html_files/static"

        self.controller = controller

    def render_GET(self, request):
        ip = request.getClientIP()
        print "FP GET: " + ip
        f = open(os.path.join(self.path_to_static_files, "index.html"))
        index = f.read()
        f.close()
        return index

    def render_POST(self, request):
        command = request.args["command"][0]
        if command:
            logger.debug("Received command: \'%s\'" % command)
            self.controller.emit("command_received", command)
        return "Success"


gobject.type_register(HtmlNetworkController)
gobject.signal_new("command_received", HtmlNetworkController, gobject.SIGNAL_RUN_FIRST, None, (gobject.TYPE_STRING, ))