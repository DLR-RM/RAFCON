
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

        self.selection = None

        view.editor.connect('expose_event', self._on_expose_event)
        view.editor.connect('button-press-event', self._on_mouse_press)
        view.editor.connect('button-release-event', self._on_mouse_release)


    def register_view(self, view):
        """Called when the View was registered
        """


    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def _on_expose_event(self, *args):
        box1 = [self.view.editor.left, self.view.editor.right, self.view.editor.top, self.view.editor.bottom]
        self.view.editor.expose_init(args)
        self.draw_state(self.model)
        self.view.editor.expose_finish(args)
        box2 = [self.view.editor.left, self.view.editor.right, self.view.editor.top, self.view.editor.bottom]

        # Calculate viewport offset from desired one
        # If too big, configure and redraw
        diff = sum(map(lambda i1, i2: abs(i1 - i2), box1, box2))
        if diff > 5:
            self.redraw()

    def redraw(self):
        self.view.editor.emit("configure_event", None)
        self.view.editor.emit("expose_event", None)


    def _on_mouse_press(self, widget, event):
        if event.button == 1:
            print 'press', event
            new_selection = self._find_selection(event.x, event.y)
            if new_selection != self.selection:
                if self.selection != None:
                    self.selection.meta['gui']['selected'] = False
                self.selection = new_selection
                if self.selection != None:
                    self.selection.meta['gui']['selected'] = True
                self.redraw()

    def _on_mouse_release(self, widget, event):
        print 'release', event

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

        active = state.meta['gui']['selected']

        id = self.view.editor.draw_state(state.state.name, pos_x, pos_y, width, height, active)
        state.meta['gui']['editor']['id'] = id

        if depth == 1:
            margin = min(width, height) / 10.0
            self.view.editor.left = pos_x - margin
            self.view.editor.right = pos_x + width + margin
            self.view.editor.bottom = pos_y - margin
            self.view.editor.top = pos_x + height + margin

        if isinstance(state, ContainerStateModel) and depth < self.max_depth:
            # iterate over child states, transitions and data flows

            state_ctr = 0
            margin = width / float(25)

            for child_state in state.states:
                state_ctr += 1

                child_width = width / float(5)
                child_height = height / float(5)

                child_pos_x = pos_x + state_ctr * margin
                child_pos_y = pos_y + height - child_height - state_ctr * margin

                self.draw_state(child_state, child_pos_x, child_pos_y, child_width, child_height, depth + 1)

            for transition in state.transitions:
                pass

            for data_flow in state.data_flows:
                pass

    def _find_selection(self, pos_x, pos_y):
        # e.g. sets render mode to GL_SELECT
        self.view.editor.prepare_selection(pos_x, pos_y)
        # draw again
        self.view.editor.expose_init()
        self.draw_state(self.model)
        self.view.editor.expose_finish()
        # get result
        hits = self.view.editor.find_selection()

        # extract ids
        selected_ids = map(lambda hit: hit[2][1], hits)
        print selected_ids
        (selection, selection_depth) = self._selection_ids_to_model(selected_ids, self.model, 1, None, 0)
        print selection, selection_depth
        return selection

    def _selection_ids_to_model(self, ids, search_state, search_state_depth, selection, selection_depth):
        # Only the element which is furthest down in the hierarchy is selected
        if search_state_depth > selection_depth:
            # Check whether the id of the current state matches an id in the selected ids
            if search_state.meta['gui']['editor']['id'] and search_state.meta['gui']['editor']['id'] in ids:
                print "possible selection", search_state
                # if so, add the state to the list of selected states
                selection = search_state
                selection_depth = search_state_depth
                # remove the id from the list to fasten up further searches
                ids.remove(search_state.meta['gui']['editor']['id'])

        # Return if there is nothing more to find
        if len(ids) == 0:
            return selection, selection_depth

        # If it is a container state, check its transitions, data flows and child states
        if isinstance(search_state, ContainerStateModel):

            for state in search_state.states:
                (selection, selection_depth) = self._selection_ids_to_model(ids, state, search_state_depth + 1,
                                                                            selection, selection_depth)

            if len(ids) == 0 or search_state_depth < selection_depth:
                return selection, selection_depth

            for transition in search_state.transitions:
                if transition.meta['gui']['editor']['id'] and transition.meta['gui']['editor']['id'] in ids:
                    selection = transition
                    ids.remove(transition.meta['gui']['editor']['id'])

            if len(ids) == 0:
                return selection, selection_depth

            for data_flow in search_state.data_flows:
                if data_flow.meta['gui']['editor']['id'] and data_flow.meta['gui']['editor']['id'] in ids:
                    selection = data_flow
                    ids.remove(data_flow.meta['gui']['editor']['id'])

        return selection, selection_depth