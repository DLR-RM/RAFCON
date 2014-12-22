import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

from utils import log

logger = log.get_logger(__name__)

from gtkmvc import Controller
from mvc.models import ContainerStateModel, StateModel
# from models.container_state import ContainerStateModel


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
        self.selection_start_pos = (0, 0)
        self.mouse_move_start_pos = (0, 0)

        view.editor.connect('expose_event', self._on_expose_event)
        view.editor.connect('button-press-event', self._on_mouse_press)
        view.editor.connect('button-release-event', self._on_mouse_release)
        # Only called when the mouse is clicked while moving
        view.editor.connect('motion-notify-event', self._on_mouse_motion)

    def register_view(self, view):
        """Called when the View was registered
        """
        pass

    def register_adapters(self):
        """Adapters should be registered in this method call
        """
        pass

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
            # print 'press', event
            self.mouse_move_start_pos = (event.x, event.y)
            new_selection = self._find_selection(event.x, event.y)
            if new_selection != self.selection:
                if self.selection is not None:
                    self.selection.meta['gui']['selected'] = False
                self.selection = new_selection
                if self.selection is not None:
                    self.selection.meta['gui']['selected'] = True
                self.redraw()
            if self.selection is not None and isinstance(self.selection, StateModel):
                self.selection_start_pos = (self.selection.meta['gui']['editor']['pos_x'],
                                            self.selection.meta['gui']['editor']['pos_y'])

    def _on_mouse_release(self, widget, event):
        # print 'release', event
        pass

    def _on_mouse_motion(self, widget, event):
        if self.selection is None:
            return
        if not isinstance(self.selection, StateModel):
            return
        if self.selection == self.model:
            return
        # print 'motion', event
        rel_x_motion = event.x - self.mouse_move_start_pos[0]
        rel_y_motion = -(event.y - self.mouse_move_start_pos[1])
        if self.selection is not None and isinstance(self.selection, StateModel):
            conversion = self.view.editor.pixel_to_size_ratio()

            new_pos_x = self.selection_start_pos[0] + rel_x_motion / conversion
            new_pos_y = self.selection_start_pos[1] + rel_y_motion / conversion

            cur_width = self.selection.meta['gui']['editor']['width']
            cur_height = self.selection.meta['gui']['editor']['height']

            if new_pos_x < self.selection.parent.meta['gui']['editor']['pos_x']:
                new_pos_x = self.selection.parent.meta['gui']['editor']['pos_x']
            elif new_pos_x + cur_width > self.selection.parent.meta['gui']['editor']['pos_x'] + \
                                         self.selection.parent.meta['gui']['editor']['width']:
                new_pos_x = self.selection.parent.meta['gui']['editor']['pos_x'] + \
                            self.selection.parent.meta['gui']['editor']['width'] - cur_width

            if new_pos_y < self.selection.parent.meta['gui']['editor']['pos_y']:
                new_pos_y = self.selection.parent.meta['gui']['editor']['pos_y']
            elif new_pos_y + cur_height > self.selection.parent.meta['gui']['editor']['pos_y'] + \
                                         self.selection.parent.meta['gui']['editor']['height']:
                new_pos_y = self.selection.parent.meta['gui']['editor']['pos_y'] + \
                            self.selection.parent.meta['gui']['editor']['height'] - cur_height

            self.selection.meta['gui']['editor']['pos_x'] = new_pos_x
            self.selection.meta['gui']['editor']['pos_y'] = new_pos_y

            self.redraw()

    def draw_state(self, state, pos_x=0, pos_y=0, width=100, height=100, depth=1):
        assert isinstance(state, StateModel)

        if not state.meta['gui']['editor']['width']:
            state.meta['gui']['editor']['width'] = width
        if not state.meta['gui']['editor']['height']:
            state.meta['gui']['editor']['height'] = height

        width = state.meta['gui']['editor']['width']
        height = state.meta['gui']['editor']['height']

        if not state.meta['gui']['editor']['pos_x'] and state.meta['gui']['editor']['pos_x'] != 0:
            state.meta['gui']['editor']['pos_x'] = pos_x
        if not state.meta['gui']['editor']['pos_y'] and state.meta['gui']['editor']['pos_y'] != 0:
            state.meta['gui']['editor']['pos_y'] = pos_y

        pos_x = state.meta['gui']['editor']['pos_x']
        pos_y = state.meta['gui']['editor']['pos_y']

        active = state.meta['gui']['selected']

        (id, outcome_pos) = self.view.editor.draw_state(state.state.name, pos_x, pos_y, width, height,
                                                        state.state.outcomes, active, depth)
        state.meta['gui']['editor']['id'] = id
        state.meta['gui']['editor']['outcome_pos'] = outcome_pos

        if depth == 1:
            margin = min(width, height) / 10.0
            self.view.editor.left = pos_x - margin
            self.view.editor.right = pos_x + width + margin
            self.view.editor.bottom = pos_y - margin
            self.view.editor.top = pos_y + height + margin

        if isinstance(state, ContainerStateModel) and depth < self.max_depth:
            # iterate over child states, transitions and data flows

            state_ctr = 0
            margin = width / float(25)

            for child_state in state.states.itervalues():
                state_ctr += 1

                child_width = width / float(5)
                child_height = height / float(5)

                child_pos_x = pos_x + state_ctr * margin
                child_pos_y = pos_y + height - child_height - state_ctr * margin

                self.draw_state(child_state, child_pos_x, child_pos_y, child_width, child_height,
                                depth + (1.0 / len(state.states)) + 0.5)

            for transition in state.transitions:
                from_state_id = transition.transition.from_state
                to_state_id = transition.transition.to_state
                from_state = None
                to_state = None
                for child_state in state.states.itervalues():
                    if child_state.state.state_id == from_state_id:
                        from_state = child_state
                    if child_state.state.state_id == to_state_id:
                        to_state = child_state

                assert isinstance(from_state, StateModel), "Transition from unknown state with ID {id:s}".format(
                    id=from_state_id)

                try:
                    from_x = state.states[from_state_id].meta['gui']['editor']['outcome_pos'][
                        transition.transition.from_outcome][0]
                    from_y = state.states[from_state_id].meta['gui']['editor']['outcome_pos'][
                        transition.transition.from_outcome][1]
                except Exception as e:
                    logger.error("""Outcome position was not found. \
                            maybe the outcome for teh transition was not found: {err}""".format(err=e))
                    continue

                if to_state is None:  # Transition goes back to parent
                    to_x = state.meta['gui']['editor']['outcome_pos'][transition.transition.to_outcome][0]
                    to_y = state.meta['gui']['editor']['outcome_pos'][transition.transition.to_outcome][1]
                else:
                    to_x = to_state.meta['gui']['editor']['pos_x'] + to_state.meta['gui']['editor']['width'] / 2
                    to_y = to_state.meta['gui']['editor']['pos_y'] + to_state.meta['gui']['editor']['height'] / 2

                active = transition.meta['gui']['selected']
                id = self.view.editor.draw_transition("Transition", from_x, from_y, to_x, to_y, active, depth + 0.5)
                transition.meta['gui']['editor']['id'] = id

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
        selection = None
        try:
            selected_ids = map(lambda hit: hit[2][1], hits)
            # print selected_ids
            (selection, selection_depth) = self._selection_ids_to_model(selected_ids, self.model, 1, None, 0)
            # print selection, selection_depth
        except Exception as e:
            logger.error("Error while finding selection: {err:s}".format(err=e))
            pass
        return selection

    def _selection_ids_to_model(self, ids, search_state, search_state_depth, selection, selection_depth):
        # Only the element which is furthest down in the hierarchy is selected
        if search_state_depth > selection_depth:
            # Check whether the id of the current state matches an id in the selected ids
            if search_state.meta['gui']['editor']['id'] and search_state.meta['gui']['editor']['id'] in ids:
                # print "possible selection", search_state
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

            for state in search_state.states.itervalues():
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