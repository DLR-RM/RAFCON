"""
.. module:: web_server
   :synopsis: Serves the built web GUI as static files next to the websocket server

Uses only the standard library. The web app fetches ``/config.json`` on startup to learn the
websocket port of the RAFCON network server.
"""

import json
import os
import posixpath
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from rafcon.network import protocol

from rafcon.utils import log
logger = log.get_logger(__name__)

DEFAULT_WEB_PORT = 8880

MISSING_DIST_PAGE = (
    "The RAFCON web GUI has not been built yet.\n\n"
    "Build it with:\n"
    "    cd source/rafcon/web/app\n"
    "    npm install\n"
    "    npm run build\n"
)


def default_dist_path():
    from importlib import resources
    return str(resources.files("rafcon.web") / "dist")


class _WebAppRequestHandler(SimpleHTTPRequestHandler):
    """Static file handler with a dynamic /config.json and SPA fallback to index.html"""

    server_version = "RAFCONWebServer"

    def do_GET(self):
        if self.path.split("?")[0] == "/config.json":
            return self._serve_config()
        if not os.path.isdir(self.directory):
            return self._serve_missing_dist()
        return super(_WebAppRequestHandler, self).do_GET()

    def send_head(self):
        # SPA fallback: unknown non-asset paths serve index.html
        path = self.translate_path(self.path)
        if not os.path.exists(path) and not posixpath.splitext(self.path.split("?")[0])[1]:
            self.path = "/index.html"
        return super(_WebAppRequestHandler, self).send_head()

    def _serve_config(self):
        body = json.dumps({"websocket_port": self.server.rafcon_ws_port,
                           "protocol_version": protocol.PROTOCOL_VERSION}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_missing_dist(self):
        body = MISSING_DIST_PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        logger.debug("Web GUI request: " + format % args)


class WebServer:
    """Serves the built web GUI

    :param int port: HTTP port to listen on
    :param int ws_port: port of the running RAFCON network (websocket) server, told to clients
    :param str dist_path: folder with the built web app; defaults to the packaged ``rafcon/web/dist``
    """

    def __init__(self, port=DEFAULT_WEB_PORT, ws_port=protocol.DEFAULT_PORT, dist_path=None):
        self.port = port
        self.ws_port = ws_port
        self.dist_path = dist_path or default_dist_path()
        self._httpd = None
        self._thread = None

    def start(self):
        dist_path = self.dist_path

        class Handler(_WebAppRequestHandler):
            def __init__(self, *args, **kwargs):
                super(Handler, self).__init__(*args, directory=dist_path, **kwargs)

        self._httpd = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        self._httpd.rafcon_ws_port = self.ws_port
        self._thread = threading.Thread(target=self._httpd.serve_forever, name="RAFCONWebServer", daemon=True)
        self._thread.start()
        logger.info("RAFCON web GUI available at http://localhost:{0}".format(self.port))

    def stop(self):
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
        if self._thread:
            self._thread.join(3)
            self._thread = None
