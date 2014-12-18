
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller


class GraphicalEditorController(Controller):
    """Controller handling the graphical editor

    :param mvc.models.ContainerStateModel model: The root container state model containing the data
    :param mvc.views.GraphicalEditorView view: The GTK view having an OpenGL rendering element
    """

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

        view.editor.connect('expose_event',    self._on_expose_event)


    def register_view(self, view):
        """Called when the View was registered
        """


    def register_adapters(self):
        """Adapters should be registered in this method call
        """


    def _on_expose_event(self, *args):
        self.view.editor.expose_init(args)

        self.draw_container(self.model)

        self.view.editor.expose_finish(args)

    def draw_container(self, container):

        if not container.meta['gui']['editor']['width']:
            container.meta['gui']['editor']['width'] = 100
        if not container.meta['gui']['editor']['height']:
            container.meta['gui']['editor']['height'] = 100

        width = container.meta['gui']['editor']['width']
        height = container.meta['gui']['editor']['height']

        self.view.editor.draw_container(container.container_state.name, width, height)

        # iterate over child states, transitions and data flows

        