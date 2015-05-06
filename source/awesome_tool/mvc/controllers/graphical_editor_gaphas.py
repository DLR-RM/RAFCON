from awesome_tool.utils import log
logger = log.get_logger(__name__)

from awesome_tool.mvc.controllers.extended_controller import ExtendedController

from awesome_tool.mvc.models.state_machine import StateMachineModel
from awesome_tool.mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel

from awesome_tool.mvc.views.graphical_editor_gaphas import GraphicalEditorView, StateView

from gaphas import Canvas
from gaphas.matrix import Matrix
import gaphas.guide

class GraphicalEditorController(ExtendedController):
    """Controller handling the graphical editor

    :param awesome_tool.mvc.models.state_machine.StateMachineModel model: The state machine model, holding the root
        state and the current selection
    :param awesome_tool.mvc.views.graphical_editor.GraphicalEditorView view: The GTK view having an OpenGL rendering
        element
    """

    def __init__(self, model, view):
        """Constructor
        """
        ExtendedController.__init__(self, model, view)
        print "view", type(view), GraphicalEditorView
        assert type(view) == GraphicalEditorView
        assert isinstance(self.model, StateMachineModel)
        # assert isinstance(self.view, GraphicalEditorView)
        # assert isinstance(self.view.editor, GraphicalEditor)
        self.root_state_m = model.root_state

        self.canvas = Canvas()
        self.zoom = 3.

    def register_view(self, view):
        """Called when the View was registered
        """
        assert self.view == view
        self.setup_canvas()
        self.view.setup_canvas(self.canvas, self.zoom)
        # print self.view.editor.matrix
        # # self.view.editor.matrix = self.translation_matrix((10, 10))
        # hadjustment = self.view.editor.hadjustment
        # vadjustment = self.view.editor.vadjustment
        # print hadjustment.get_value(), hadjustment.get_lower(), hadjustment.get_upper()
        # hadjustment.set_lower(-10)
        # hadjustment.set_value(0)
        # hadjustment.set_upper()
        # print hadjustment.get_value()
        #
        # vadjustment.set_value(hadjustment.get_value() + 100)
        # self.view.editor.on_adjustment_changed(hadjustment)
        # self.view.editor.on_adjustment_changed(vadjustment)
        # # self.view.editor.do_set_scroll_adjustments(hadjustment, vadjustment)
        # print self.view.editor.matrix

    def register_adapters(self):
        """Adapters should be registered in this method call
        """
        pass

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param awesome_tool.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        pass

    @ExtendedController.observe("state_machine", after=True)
    def state_machine_change(self, model, prop_name, info):
        """Called on any change within th state machine

        This method is called, when any state, transition, data flow, etc. within the state machine changes. This
        then typically requires a redraw of the graphical editor, to display these changes immediately.

        :param awesome_tool.mvc.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The property that was changed
        :param dict info: Information about the change
        """
        # if 'method_name' in info and info['method_name'] == 'root_state_after_change':
        #     self._redraw()
        pass

    @ExtendedController.observe("root_state", assign=True)
    def root_state_change(self, model, prop_name, info):
        """Called when the root state was exchanged

        Exchanges the local reference to the root state and redraws.

        :param awesome_tool.mvc.models.state_machine.StateMachineModel model: The state machine model
        :param str prop_name: The root state
        :param dict info: Information about the change
        """
        if self.root_state_m is not model.root_state:
            logger.debug("The root state was exchanged")
            self.root_state_m = model.root_state


    @ExtendedController.observe("selection", after=True)
    def selection_change(self, model, prop_name, info):
        """Called when the selection was changed externally

        Updates the local selection and redraws.

        :param awesome_tool.mvc.selection.Selection model: The state machine model
        :param str prop_name: The selection
        :param dict info: Information about the change
        """
        pass

    def setup_canvas(self):

        self.setup_state(self.root_state_m)

    def setup_state(self, state_m, parent=None, rel_pos=(10, 10), size=(100, 100)):

        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.

        :param awesome_tool.mvc.models.state.StateModel state_m: The state to be drawn
        :param tuple rel_pos: The default relative position (x, y) if there is no relative position stored
        :param tuple size: The default size (width, height) if there is no size stored
        :param float depth: The hierarchy level of the state
        """
        assert isinstance(state_m, StateModel)
        state_meta = state_m.meta['gui']['editor']
        state_temp = state_m.temp['gui']['editor']

        # Use default values if no size information is stored
        if not isinstance(state_meta['size'], tuple):
            state_meta['size'] = size

        size = state_meta['size']

        if isinstance(state_meta['rel_pos'], tuple):
            rel_pos = state_meta['rel_pos']
            if not isinstance(state_meta['invert_y'], bool) or state_meta['invert_y']:
                rel_pos = (rel_pos[0], -rel_pos[1])

        # Was the state selected?
        selected_states = self.model.selection.get_states()
        selected = False if state_m not in selected_states else True

        # Is the state active (executing)?
        active = 0
        if state_m.state.active:
            if self.has_content(state_m) and state_m.state.child_execution:
                active = 0.5
            else:
                active = 1

        state_v = StateView(size)
        state_v.matrix.translate(*rel_pos)

        self.canvas.add(state_v, parent)

        if parent is not None:
            # Keep state within parent
            pass

        if isinstance(state_m, ContainerStateModel):
            num_child_state = 0
            width = size[0]
            height = size[1]

            for child_state in state_m.states.itervalues():
                # Calculate default positions for the child states
                # Make the inset from the top left corner

                child_width = width / 5.
                child_height = height / 5.
                child_size = (child_width, child_height)
                child_spacing = max(child_size) * 1.2

                max_cols = width // child_spacing
                (row, col) = divmod(num_child_state, max_cols)
                child_rel_pos_x = col * child_spacing + child_spacing - child_width
                child_rel_pos_y = child_spacing * (1.5 * row + 1)
                child_rel_pos = (child_rel_pos_x, child_rel_pos_y)
                num_child_state += 1

                self.setup_state(child_state, state_v, child_rel_pos, child_size)

            # if global_gui_config.get_config_value('show_data_flows', True):
            #     self.draw_inner_data_ports(state_m, depth)
            #
            # self.draw_transitions(state_m, depth)
            #
            # if global_gui_config.get_config_value('show_data_flows', True):
            #     self.draw_data_flows(state_m, depth)

        # self._handle_new_transition(state_m, depth)
        #
        # if global_gui_config.get_config_value('show_data_flows', True):
        #     self._handle_new_data_flow(state_m, depth)

    @staticmethod
    def translation_matrix(translation):
        matrix = Matrix()
        matrix.translate(translation[0], translation[1])
        return matrix
