
import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller
from mvc.models import ContainerStateModel, StateModel
#from models.container_state import ContainerStateModel

class GraphicalEditorController(Controller):
    """Controller handling the graphical editor

    :param mvc.models.ContainerStateModel model: The root container state model containing the data
    :param mvc.views.GraphicalEditorView view: The GTK view having an OpenGL rendering element
    """

    max_depth = 2


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

        self.draw_state(self.model)

        self.view.editor.expose_finish(args)

    def draw_state(self, state, pos_x=0, pos_y=0, width=100, height=100, depth=1):
        assert isinstance(state, StateModel)

        if not state.meta['gui']['editor']['width']:
            state.meta['gui']['editor']['width'] = width
        if not state.meta['gui']['editor']['height']:
            state.meta['gui']['editor']['height'] = height

        width = state.meta['gui']['editor']['width']
        height = state.meta['gui']['editor']['height']

        if not state.meta['gui']['editor']['pos_x']:
            state.meta['gui']['editor']['pos_x'] = pos_x
        if not state.meta['gui']['editor']['pos_y']:
            state.meta['gui']['editor']['pos_y'] = pos_y

        pos_x = state.meta['gui']['editor']['pos_x']
        pos_y = state.meta['gui']['editor']['pos_y']

        self.view.editor.draw_container(state.state.name, pos_x, pos_y, width, height)

        if isinstance(state, ContainerStateModel) and depth < self.max_depth:
            # iterate over child states, transitions and data flows

            state_ctr = 0
            margin = width / float(25)
            print "Depth", depth, "Num childen = ", len(state.states), "ID", state.state.state_id
            for state in state.states:
                print "Child", state
                state_ctr += 1

                child_width = width / float(5)
                child_height = height / float(5)

                child_pos_x = pos_x + state_ctr * margin
                child_pos_y = pos_y + height - child_height - state_ctr * margin

                self.draw_state(state, child_pos_x, child_pos_y, child_width, child_height, depth + 1)
                # if not state.meta['gui']['editor']['width']:
                #     state.meta['gui']['editor']['width'] =
                # if not state.meta['gui']['editor']['height']:
                #     state.meta['gui']['editor']['height'] = container_height / float(5)
                #
                # width = state.meta['gui']['editor']['width']
                # height = state.meta['gui']['editor']['height']
                #
                # if not state.meta['gui']['editor']['pos_x'] or not state.meta['gui']['editor']['pos_y']:
                #     state_ctr += 1
                #
                #     state.meta['gui']['editor']['pos_x'] = container_pos_x + state_ctr * margin
                #     state.meta['gui']['editor']['pos_y'] = container_pos_y + container_height - height - \
                #                                            state_ctr * margin
                #
                # pos_x = state.meta['gui']['editor']['pos_x']
                # pos_y = state.meta['gui']['editor']['pos_y']
                # print 'pos_x', pos_x, 'pos y', pos_y, 'height', height, 'width', width
                # self.view.editor.draw_container(state.state.name, pos_x, pos_y, width, height, True)