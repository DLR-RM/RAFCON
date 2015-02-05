import OpenGL
from OpenGL.GL import *
from OpenGL.GLU import *

from utils import log

logger = log.get_logger(__name__)
import sys
import time
from gtkmvc import Controller
from mvc.models import ContainerStateModel, StateModel, TransitionModel, DataFlowModel
from math import sqrt
from gtk.gdk import SCROLL_DOWN, SCROLL_UP, SCROLL_LEFT, SCROLL_RIGHT
from gtk.gdk import keyval_name
# from models.container_state import ContainerStateModel


class GraphicalEditorController(Controller):
    """Controller handling the graphical editor

    :param mvc.models.ContainerStateModel model: The root container state model containing the data
    :param mvc.views.GraphicalEditorView view: The GTK view having an OpenGL rendering element
    """

    max_depth = 300

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

        self.selection = None
        self.selection_start_pos = (0, 0)
        self.mouse_move_start_pos = (0, 0)
        self.last_button_pressed = -1

        self.selected_outcome = None
        self.selected_waypoint = None
        self.selected_resizer = None

        self.shift_modifier = False
        self.alt_modifier = False
        self.ctrl_modifier = False

        view.editor.connect('expose_event', self._on_expose_event)
        view.editor.connect('button-press-event', self._on_mouse_press)
        view.editor.connect('button-release-event', self._on_mouse_release)
        # Only called when the mouse is clicked while moving
        view.editor.connect('motion-notify-event', self._on_mouse_motion)
        view.editor.connect('scroll-event', self._on_scroll)
        view.editor.connect('key-press-event', self._on_key_press)
        view.editor.connect('key-release-event', self._on_key_release)
        self.last_time = time.time()

    def register_view(self, view):
        """Called when the View was registered
        """
        pass

    def register_adapters(self):
        """Adapters should be registered in this method call
        """
        pass

    @Controller.observe("state", after=True)
    @Controller.observe("states", after=True)
    @Controller.observe("transitions", after=True)
    @Controller.observe("data_flows", after=True)
    def state_machine_change(self, model, property, info):
        self._redraw(True)

    def _on_expose_event(self, *args):
        """Redraw the graphical editor

        This method is called typically when the editor window is resized or something triggers are redraw. This
        controller class handles the logic for redrawing, while the corresponding view handles the design.
        :param args: console arguments, not used
        """

        # Prepare the drawing process
        self.view.editor.expose_init(args)
        # The whole logic of drawing is triggered by calling the root state to be drawn
        self.draw_state(self.model)
        # Finish the drawing process (e.g. swap buffers)
        self.view.editor.expose_finish(args)

    def _redraw(self, important=False):
        """Force the graphical editor to be redrawn

        First triggers the configure event to cause the perspective to be updated, then trigger the actual expose
        event to redraw.
        """
        # Check if initialized
        if hasattr(self.view, "editor") and (time.time() - self.last_time > 1 / 50. or important):
            self.view.editor.emit("configure_event", None)
            self.view.editor.emit("expose_event", None)
            self.last_time = time.time()

    def _on_key_press(self, widget, event):
        key_name = keyval_name(event.keyval)
        # print "key press", key_name
        if key_name == "Control_L" or key_name == "Control_R":
            self.ctrl_modifier = True
        elif key_name == "Alt_L":
            self.alt_modifier = True
        elif key_name == "Shift_L" or key_name == "Shift_R":
            self.shift_modifier = True

    def _on_key_release(self, widget, event):
        key_name = keyval_name(event.keyval)
        # print "key release", key_name
        if key_name == "Control_L" or key_name == "Control_R":
            self.ctrl_modifier = False
        elif key_name == "Alt_L":
            self.alt_modifier = False
        elif key_name == "Shift_L" or key_name == "Shift_R":
            self.shift_modifier = False

    def _on_mouse_press(self, widget, event):
        """Triggered when the mouse is pressed

        Different actions can result from a mouse click, e. g. selecting or drag'n'drop
        :param widget: The widget beneath the mouse when the click was done
        :param event: Information about the event, e. g. x and y coordinate
        """

        self.last_button_pressed = event.button
        self.selected_waypoint = None  # reset
        self.selected_outcome = None  # reset
        self.selected_resizer = None  # reset

        # Store the coordinates of the event
        self.mouse_move_start_pos = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
        self.mouse_move_last_pos = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

        if event.button == 1:  # Left mouse button
            # print 'press', event


            # Check if something was selected
            new_selection = self._find_selection(event.x, event.y)

            # Check whether a state, a transition or data flow was clicked on
            # If so, set the meta data of the object to "object selected" and redraw to highlight the object
            # If the object was previously selected, remove the selection
            if new_selection != self.selection:
                if self.selection is not None:
                    self.selection.meta['gui']['selected'] = False
                self.selection = new_selection
                if self.selection is not None:
                    self.selection.meta['gui']['selected'] = True
                    # else:
                    # self.selection.meta['gui']['selected'] = False
                    # self.selection = None

            # If a state was clicked on, store the click coordinates for the drag'ndrop movement
            if self.selection is not None and isinstance(self.selection, StateModel):
                self.selection_start_pos = (self.selection.meta['gui']['editor']['pos_x'],
                                            self.selection.meta['gui']['editor']['pos_y'])

            # Check, whether a waypoint was clicked on
            if self.selection is not None and \
                    (isinstance(self.selection, TransitionModel) or isinstance(self.selection, DataFlowModel)):
                close_threshold = min(self.selection.parent.meta['gui']['editor']['height'],
                                      self.selection.parent.meta['gui']['editor']['width)']) / 50.
                click = self.mouse_move_start_pos
                for i, waypoint in enumerate(self.selection.meta['gui']['editor']['waypoints']):
                    if waypoint[0] is not None and waypoint[1] is not None:
                        dist = sqrt((waypoint[0] - click[0]) ** 2 + (waypoint[1] - click[1]) ** 2)
                        if dist < close_threshold:
                            self.selected_waypoint = (self.selection.meta['gui']['editor']['waypoints'], i)
                            self.selection_start_pos = (waypoint[0], waypoint[1])
                            logger.debug('Selected waypoint {0:.1f} - {1:.1f}'.format(click[0], click[1]))
                            break

            # Check, whether an outcome was clicked on
            if self.selection is not None and isinstance(self.selection,
                                                         StateModel) and self.selection is not self.model:
                outcomes_close_threshold = self.selection.meta['gui']['editor']['outcome_radius']
                outcomes = self.selection.meta['gui']['editor']['outcome_pos']
                click = self.mouse_move_start_pos
                for key in outcomes:
                    dist = sqrt((outcomes[key][0] - click[0]) ** 2 + (outcomes[key][1] - click[1]) ** 2)
                    if dist < outcomes_close_threshold:
                        self.selected_outcome = (outcomes, key)
                pass

            # Check, whether a resizer was clicked on
            if self.selection is not None and isinstance(self.selection, StateModel) and self.selection:
                state_editor_data = self.selection.meta['gui']['editor']
                p1 = (state_editor_data['pos_x'] + state_editor_data['width'], state_editor_data['pos_y'])
                p2 = (p1[0] - state_editor_data['resize_length'], p1[1])
                p3 = (p1[0], p1[1] + state_editor_data['resize_length'])

                def point_in_triangle(p, v1, v2, v3):
                    def _test(p1, p2, p3):
                        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

                    b1 = _test(p, v1, v2) < 0.0
                    b2 = _test(p, v2, v3) < 0.0
                    b3 = _test(p, v3, v1) < 0.0

                    return (b1 == b2) and (b2 == b3)

                if point_in_triangle(self.mouse_move_start_pos, p1, p2, p3):
                    self.selected_resizer = self.selection
                    # Start resize process
                    pass

            self._redraw()

        if event.button == 3:  # right mouse button

            # Check if something was selected
            click = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
            clicked_model = self._find_selection(event.x, event.y)

            # If a connection (transition or data flow) was clicked
            if isinstance(clicked_model, TransitionModel) or isinstance(clicked_model, DataFlowModel):
                connection_model = clicked_model
                # If the right click was on a waypoint of a connection, the waypoint is removed
                # If it was on a connection, a waypoint is added to that position
                waypoint_removed = False

                close_threshold = min(clicked_model.parent.meta['gui']['editor']['height'],
                                      clicked_model.parent.meta['gui']['editor']['width)']) / 70.
                # logger.debug('Examining waypoint for click {0:.1f} - {1:.1f}'.format(click[0], click[1]))
                for waypoint in connection_model.meta['gui']['editor']['waypoints']:
                    if waypoint[0] is not None and waypoint[1] is not None:
                        if sqrt((waypoint[0] - click[0]) ** 2 + (waypoint[1] - click[1]) ** 2) < close_threshold:
                            connection_model.meta['gui']['editor']['waypoints'].remove(waypoint)
                            waypoint_removed = True
                            logger.debug('Connection waypoint removed')
                            self._redraw()
                            break

                if not waypoint_removed:
                    # Add waypoint
                    if isinstance(connection_model.meta['gui']['editor']['waypoints'], dict):
                        logger.warn("Connection waypoints was of type dict, expected list")
                        connection_model.meta['gui']['editor']['waypoints'] = connection_model.meta['waypoints'].items()
                    num = len(connection_model.meta['gui']['editor']['waypoints'])

                    points = [(connection_model.meta['gui']['editor']['from_pos_x'],
                               connection_model.meta['gui']['editor']['from_pos_y'])]
                    points.extend(connection_model.meta['gui']['editor']['waypoints'])
                    points.append((connection_model.meta['gui']['editor']['to_pos_x'],
                                   connection_model.meta['gui']['editor']['to_pos_y']))
                    for i in range(len(points) - 1):
                        if self._point_on_line(click, points[i], points[i + 1]):
                            connection_model.meta['gui']['editor']['waypoints'].insert(i, (click[0], click[1]))

                    logger.debug('Connection waypoint added at {0:.1f} - {1:.1f}'.format(click[0], click[1]))
                    self._redraw()


    def _on_mouse_release(self, widget, event):
        """Triggered when a mouse button is being released

        :param widget: The widget beneath the mouse when the release was done
        :param event: Information about the event, e. g. x and y coordinate
        Not used so far
        """
        # print 'release', event
        self.last_button_pressed = None

        if self.selected_outcome is not None:
            release_selection = self._find_selection(event.x, event.y, True)
            if isinstance(release_selection, StateModel) and release_selection != self.selection:
                target_state_id = None
                target_outcome = None
                if release_selection == self.selection.parent:
                    # Check whether the mouse was released on an outcome
                    outcomes_close_threshold = self.selection.parent.meta['gui']['editor']['outcome_radius']
                    outcomes = self.selection.parent.meta['gui']['editor']['outcome_pos']
                    click = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
                    for key in outcomes:
                        dist = sqrt((outcomes[key][0] - click[0]) ** 2 + (outcomes[key][1] - click[1]) ** 2)
                        if dist < outcomes_close_threshold:
                            # This is a possible connection:
                            # The outcome of a state is connected to ian outcome of its parent state
                            target_outcome = key

                elif release_selection.parent == self.selection.parent:
                    # This is a possible connection:
                    # The outcome of a state is connected to another state, which is on the same hierarchy
                    target_state_id = release_selection.state.state_id

                if target_state_id is not None or target_outcome is not None:
                    state_id = self.selection.state.state_id
                    outcome_id = self.selected_outcome[1]
                    try:
                        self.selection.parent.state.add_transition(state_id, outcome_id,
                                                                   target_state_id, target_outcome)
                    except AttributeError as e:
                        logger.debug("Transition couldn't be added: {0}".format(e))
                    except Exception as e:
                        logger.error("Unexpected exception: {0}".format(e))

            self.selected_outcome = None
            self._redraw(True)

    def _on_mouse_motion(self, widget, event):
        """Triggered when the mouse is moved while being pressed

        When a state is selected, this causes a drag'n'drop movement
        :param widget: The widget beneath the mouse when the click was done
        :param event: Information about the event, e. g. x and y coordinate
        """

        mouse_current_pos = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))
        rel_x_motion = mouse_current_pos[0] - self.mouse_move_start_pos[0]
        rel_y_motion = mouse_current_pos[1] - self.mouse_move_start_pos[1]

        # Move while middle button is clicked moves the view
        if self.last_button_pressed == 2:
            self.view.editor.left -= rel_x_motion
            self.view.editor.right -= rel_x_motion
            self.view.editor.bottom -= rel_y_motion
            self.view.editor.top -= rel_y_motion

            self._redraw()


        # Translate the mouse movement to OpenGL coordinates
        new_pos_x = self.selection_start_pos[0] + rel_x_motion
        new_pos_y = self.selection_start_pos[1] + rel_y_motion

        def limit_pos_to_state(state, pos_x, pos_y, width=0, height=0):
            if state is not None:
                if pos_x < state.meta['gui']['editor']['pos_x']:
                    pos_x = state.meta['gui']['editor']['pos_x']
                elif pos_x + width > state.meta['gui']['editor']['pos_x'] + state.meta['gui']['editor']['width']:
                    pos_x = state.meta['gui']['editor']['pos_x'] + state.meta['gui']['editor']['width'] - width

                if pos_y < state.meta['gui']['editor']['pos_y']:
                    pos_y = state.meta['gui']['editor']['pos_y']
                elif pos_y + height > state.meta['gui']['editor']['pos_y'] + state.meta['gui']['editor']['height']:
                    pos_y = state.meta['gui']['editor']['pos_y'] + state.meta['gui']['editor']['height'] - height
            return pos_x, pos_y

        # Root container can't be moved
        if self.selection is not None and isinstance(self.selection, StateModel) and self.selection != self.model and \
                        self.last_button_pressed == 1 and self.selected_outcome is None and self.selected_resizer is None:

            old_pos_x = self.selection.meta['gui']['editor']['pos_x']
            old_pos_y = self.selection.meta['gui']['editor']['pos_y']

            cur_width = self.selection.meta['gui']['editor']['width']
            cur_height = self.selection.meta['gui']['editor']['height']

            # Keep the state within its container state
            new_pos_x, new_pos_y = limit_pos_to_state(self.selection.parent, new_pos_x, new_pos_y,
                                                      cur_width, cur_height)

            self.selection.meta['gui']['editor']['pos_x'] = new_pos_x
            self.selection.meta['gui']['editor']['pos_y'] = new_pos_y

            def move_child_states(state, diff_x, diff_y):
                # Move waypoints
                if isinstance(state, ContainerStateModel):
                    for transition in state.transitions:
                        for i, waypoint in enumerate(transition.meta['gui']['editor']['waypoints']):
                            new_pos = (waypoint[0] + diff_x, waypoint[1] + diff_y)
                            transition.meta['gui']['editor']['waypoints'][i] = new_pos
                    for data_flow in state.data_flows:
                        for i, waypoint in enumerate(data_flow.meta['gui']['editor']['waypoints']):
                            new_pos = (waypoint[0] + diff_x, waypoint[1] + diff_y)
                            data_flow.meta['gui']['editor']['waypoints'][i] = new_pos
                # Move child states
                for child_state in state.states.itervalues():
                    child_state.meta['gui']['editor']['pos_x'] += diff_x
                    child_state.meta['gui']['editor']['pos_y'] += diff_y

                    if isinstance(child_state, ContainerStateModel):
                        move_child_states(child_state, diff_x, diff_y)

            # Move all child states in accordance with the state, to keep their relative position
            if isinstance(self.selection, ContainerStateModel):
                diff_x = new_pos_x - old_pos_x
                diff_y = new_pos_y - old_pos_y
                move_child_states(self.selection, diff_x, diff_y)

            self._redraw()

        if self.selected_waypoint is not None:
            # Keep the waypoint within its container state
            new_pos_x, new_pos_y = limit_pos_to_state(self.selection.parent, new_pos_x, new_pos_y)
            self.selected_waypoint[0][self.selected_waypoint[1]] = (new_pos_x, new_pos_y)
            self._redraw()

        if self.selected_outcome is not None:
            self._redraw()

        if self.selected_resizer is not None:
            state_editor_data = self.selection.meta['gui']['editor']
            mouse_resize_pos = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

            if self.shift_modifier:
                state_size_ratio = state_editor_data['width'] / state_editor_data['height']
                if rel_x_motion / state_size_ratio < rel_y_motion:
                    mouse_resize_pos = (mouse_resize_pos[0],
                                        self.mouse_move_start_pos[1] - rel_x_motion / state_size_ratio)
                else:
                    mouse_resize_pos = (self.mouse_move_start_pos[0] - rel_y_motion * state_size_ratio,
                                        mouse_resize_pos[1])

            width = mouse_resize_pos[0] - state_editor_data['pos_x']
            height_diff = state_editor_data['pos_y'] - mouse_resize_pos[1]
            height = state_editor_data['height'] + height_diff

            min_right_edge = state_editor_data['pos_x']
            max_bottom_edge = state_editor_data['pos_y'] + state_editor_data['height']
            if not self.ctrl_modifier and isinstance(self.selection, ContainerStateModel):
                # Check lower right corner of all child states
                for child_state_m in self.selection.states.itervalues():
                    child_right_edge = child_state_m.meta['gui']['editor']['pos_x'] + \
                                       child_state_m.meta['gui']['editor']['width']
                    child_bottom_edge = child_state_m.meta['gui']['editor']['pos_y']
                    if min_right_edge < child_right_edge:
                        min_right_edge = child_right_edge
                    if max_bottom_edge > child_bottom_edge:
                        max_bottom_edge = child_bottom_edge

            # Check for parent size limitation
            max_right_edge = sys.maxint
            min_bottom_edge = -sys.maxint - 1
            if self.selection.parent is not None:
                max_right_edge = self.selection.parent.meta['gui']['editor']['pos_x'] + \
                                 self.selection.parent.meta['gui']['editor']['width']
                min_bottom_edge = self.selection.parent.meta['gui']['editor']['pos_y']

            desired_right_edge = state_editor_data['pos_x'] + width
            desired_bottom_edge = state_editor_data['pos_y'] - height_diff

            old_width = state_editor_data['width']
            old_height = state_editor_data['height']
            old_pos_x= state_editor_data['pos_x']
            old_pos_y= state_editor_data['pos_y']

            if width > 0:
                if desired_right_edge > max_right_edge:
                    state_editor_data['width'] = max_right_edge - state_editor_data['pos_x']
                elif desired_right_edge < min_right_edge:
                    state_editor_data['width'] = min_right_edge - state_editor_data['pos_x']
                else:
                    state_editor_data['width'] = width
            if height > 0:
                if desired_bottom_edge > max_bottom_edge:
                    state_editor_data['height'] += state_editor_data['pos_y'] - max_bottom_edge
                    state_editor_data['pos_y'] = max_bottom_edge
                elif desired_bottom_edge < min_bottom_edge:
                    state_editor_data['height'] += state_editor_data['pos_y'] - min_bottom_edge
                    state_editor_data['pos_y'] = min_bottom_edge
                else:
                    state_editor_data['height'] = height
                    state_editor_data['pos_y'] -= height_diff

            width_factor = state_editor_data['width'] / old_width
            height_factor = state_editor_data['height'] / old_height
            if (width_factor != 1 or height_factor != 1) and self.ctrl_modifier:

                def resize_children(state_m, width_factor, height_factor, old_pos_x, old_pos_y):
                    def calc_new_pos(old_parent_pos, new_parent_pos, old_self_pos, factor):
                        diff_pos = old_self_pos - old_parent_pos
                        diff_pos *= factor
                        return new_parent_pos + diff_pos
                    if isinstance(state_m, ContainerStateModel):
                        for transition_m in state_m.transitions:
                            for i, waypoint in enumerate(transition_m.meta['gui']['editor']['waypoints']):
                                new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                         waypoint[0], width_factor)
                                new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                         waypoint[1], height_factor)
                                transition_m.meta['gui']['editor']['waypoints'][i] = (new_pos_x, new_pos_y)
                        for data_flow_m in state_m.data_flows:
                            for i, waypoint in enumerate(data_flow_m.meta['gui']['editor']['waypoints']):
                                new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                         waypoint[0], width_factor)
                                new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                         waypoint[1], height_factor)
                                data_flow_m.meta['gui']['editor']['waypoints'][i] = (new_pos_x, new_pos_y)

                        for child_state_m in state_m.states.itervalues():
                            child_state_m.meta['gui']['editor']['width'] *= width_factor
                            child_state_m.meta['gui']['editor']['height'] *= height_factor

                            child_old_pos_x = child_state_m.meta['gui']['editor']['pos_x']
                            new_pos_x = calc_new_pos(old_pos_x, state_m.meta['gui']['editor']['pos_x'],
                                                         child_state_m.meta['gui']['editor']['pos_x'], width_factor)
                            child_state_m.meta['gui']['editor']['pos_x'] = new_pos_x

                            child_old_pos_y = child_state_m.meta['gui']['editor']['pos_y']
                            new_pos_y = calc_new_pos(old_pos_y, state_m.meta['gui']['editor']['pos_y'],
                                                         child_state_m.meta['gui']['editor']['pos_y'], height_factor)
                            child_state_m.meta['gui']['editor']['pos_y'] = new_pos_y

                            if isinstance(child_state_m, ContainerStateModel):
                                resize_children(child_state_m, width_factor, height_factor,
                                                child_old_pos_x, child_old_pos_y)

                resize_children(self.selection, width_factor, height_factor, old_pos_x, old_pos_y)
            self._redraw()

        self.mouse_move_last_pos = self.view.editor.screen_to_opengl_coordinates((event.x, event.y))

    def _on_scroll(self, widget, event):
        pos = (event.x, event.y)
        zoom_in = event.direction == SCROLL_UP
        zoom_out = event.direction == SCROLL_DOWN

        if zoom_in or zoom_out:
            old_mouse_pos = self.view.editor.screen_to_opengl_coordinates(pos)

            zoom = 1.25
            zoom = zoom if zoom_in else 1. / zoom

            # Apply centric zoom
            self.view.editor.left *= zoom
            self.view.editor.right *= zoom
            self.view.editor.bottom *= zoom
            self.view.editor.top *= zoom

            # Determine mouse offset to previous position
            aspect = self.view.editor.allocation.width / float(self.view.editor.allocation.height)
            new_mouse_pos = self.view.editor.screen_to_opengl_coordinates(pos)
            diff_x = new_mouse_pos[0] - old_mouse_pos[0]
            diff_y = new_mouse_pos[1] - old_mouse_pos[1]
            if aspect < 1:
                diff_y *= aspect
            else:
                diff_x /= aspect

            # Move view to keep the previous mouse position in the view
            self.view.editor.left -= diff_x
            self.view.editor.right -= diff_x
            self.view.editor.bottom -= diff_y
            self.view.editor.top -= diff_y

            self._redraw()


    def draw_state(self, state, pos_x=0.0, pos_y=0.0, width=100.0, height=100.0, depth=1):
        """Draws a (container) state with all its content

        Mainly contains the logic for drawing (e. g. reading and calculating values). The actual drawing process is
        done in the view, which is called from this method with the appropriate arguments.
        :param state: The state to be drawn
        :param pos_x: The default x position if there is no position stored
        :param pos_y: The default y position if there is no position stored
        :param width: The default width if there is no size stored
        :param height: The default height if there is no size stored
        :param depth: The hierarchy level of the state
        """
        assert isinstance(state, StateModel)

        # Use default values if no size information is stored
        if not state.meta['gui']['editor']['width']:
            state.meta['gui']['editor']['width'] = width
        if not state.meta['gui']['editor']['height']:
            state.meta['gui']['editor']['height'] = height

        width = state.meta['gui']['editor']['width']
        height = state.meta['gui']['editor']['height']

        # Use default values if no size information is stored
        # Here the possible case of pos_x and posy_y == 0 must be handled
        if not state.meta['gui']['editor']['pos_x'] and state.meta['gui']['editor']['pos_x'] != 0:
            state.meta['gui']['editor']['pos_x'] = pos_x
        if not state.meta['gui']['editor']['pos_y'] and state.meta['gui']['editor']['pos_y'] != 0:
            state.meta['gui']['editor']['pos_y'] = pos_y

        pos_x = state.meta['gui']['editor']['pos_x']
        pos_y = state.meta['gui']['editor']['pos_y']

        scoped_ports = {}
        if isinstance(state, ContainerStateModel):
            scoped_ports = state.state.scoped_variables

        # Was the state selected?
        active = state.meta['gui']['selected']

        # Call the drawing method of the view
        # The view returns the id of the state in OpenGL and the positions of the outcomes, input and output ports
        (id, outcome_pos, outcome_radius, input_pos, output_pos, scoped_pos, resize_length) = \
            self.view.editor.draw_state(
                state.state.name,
                pos_x, pos_y, width, height,
                state.state.outcomes,
                state.state.input_data_ports,
                state.state.output_data_ports,
                scoped_ports,
                active, depth)
        state.meta['gui']['editor']['id'] = id
        state.meta['gui']['editor']['outcome_pos'] = outcome_pos
        state.meta['gui']['editor']['outcome_radius'] = outcome_radius
        state.meta['gui']['editor']['input_pos'] = input_pos
        state.meta['gui']['editor']['output_pos'] = output_pos
        state.meta['gui']['editor']['scoped_pos'] = scoped_pos
        state.meta['gui']['editor']['resize_length'] = resize_length

        # If the state is a container state, we also have to draw its transitions and data flows as well as
        # recursively its child states
        if isinstance(state, ContainerStateModel) and depth < self.max_depth:

            state_ctr = 0
            margin = width / float(25)

            for child_state in state.states.itervalues():
                # Calculate default positions for teh child states
                # Make the inset from the top left corner
                state_ctr += 1

                child_width = width / 5.
                child_height = height / 5.

                child_pos_x = pos_x + state_ctr * margin
                child_pos_y = pos_y + height - child_height - state_ctr * margin

                self.draw_state(child_state, child_pos_x, child_pos_y, child_width, child_height,
                                depth + 1.0)#(1.0 / len(state.states)) + 0.5)

            for transition in state.transitions:
                # Get id and references to the from and to state
                from_state_id = transition.transition.from_state
                to_state_id = transition.transition.to_state
                from_state = state.states[from_state_id]
                to_state = None if to_state_id is None else state.states[to_state_id]

                assert isinstance(from_state, StateModel), "Transition from unknown state with ID {id:s}".format(
                    id=from_state_id)

                try:
                    # Set the from coordinates to the outcome coordinates received earlier
                    from_x = state.states[from_state_id].meta['gui']['editor']['outcome_pos'][
                        transition.transition.from_outcome][0]
                    from_y = state.states[from_state_id].meta['gui']['editor']['outcome_pos'][
                        transition.transition.from_outcome][1]
                except Exception as e:
                    logger.error("""Outcome position was not found. \
                            maybe the outcome for the transition was not found: {err}""".format(err=e))
                    continue

                if to_state is None:  # Transition goes back to parent
                    # Set the to coordinates to the outcome coordinates received earlier
                    to_x = state.meta['gui']['editor']['outcome_pos'][transition.transition.to_outcome][0]
                    to_y = state.meta['gui']['editor']['outcome_pos'][transition.transition.to_outcome][1]
                else:
                    # Set the to coordinates to the center of the next state
                    to_x = to_state.meta['gui']['editor']['pos_x'] + to_state.meta['gui']['editor']['width'] / 2
                    to_y = to_state.meta['gui']['editor']['pos_y'] + to_state.meta['gui']['editor']['height'] / 2

                waypoints = []
                for waypoint in transition.meta['gui']['editor']['waypoints']:
                    waypoints.append((waypoint[0], waypoint[1]))


                # Let the view draw the transition and store the returned OpenGl object id
                active = transition.meta['gui']['selected']
                line_width = min(width, height) / 25.0
                id = self.view.editor.draw_transition(from_x, from_y, to_x, to_y, line_width, waypoints,
                                                      active, depth + 0.5)
                transition.meta['gui']['editor']['id'] = id
                transition.meta['gui']['editor']['from_pos_x'] = from_x
                transition.meta['gui']['editor']['from_pos_y'] = from_y
                transition.meta['gui']['editor']['to_pos_x'] = to_x
                transition.meta['gui']['editor']['to_pos_y'] = to_y

            for data_flow in state.data_flows:
                # Get id and references to the from and to state
                from_state_id = data_flow.data_flow.from_state
                to_state_id = data_flow.data_flow.to_state
                from_state = state if from_state_id == state.state.state_id else state.states[from_state_id]
                to_state = state if to_state_id == state.state.state_id else state.states[to_state_id]

                from_key = data_flow.data_flow.from_key
                to_key = data_flow.data_flow.to_key

                connectors = dict(from_state.meta['gui']['editor']['input_pos'].items() +
                                  from_state.meta['gui']['editor']['output_pos'].items() +
                                  from_state.meta['gui']['editor']['scoped_pos'].items() +
                                  to_state.meta['gui']['editor']['input_pos'].items() +
                                  to_state.meta['gui']['editor']['output_pos'].items() +
                                  to_state.meta['gui']['editor']['scoped_pos'].items() +
                                  state.meta['gui']['editor']['input_pos'].items() +
                                  state.meta['gui']['editor']['output_pos'].items() +
                                  state.meta['gui']['editor']['scoped_pos'].items())
                from_x = connectors[from_key][0]
                from_y = connectors[from_key][1]
                to_x = connectors[to_key][0]
                to_y = connectors[to_key][1]

                waypoints = []
                for waypoint in data_flow.meta['gui']['editor']['waypoints']:
                    waypoints.append((waypoint[0], waypoint[1]))

                active = data_flow.meta['gui']['selected']
                line_width = min(width, height) / 25.0
                id = self.view.editor.draw_data_flow(from_x, from_y, to_x, to_y, line_width, waypoints,
                                                     active, depth + 0.5)
                data_flow.meta['gui']['editor']['id'] = id
                data_flow.meta['gui']['editor']['from_pos_x'] = from_x
                data_flow.meta['gui']['editor']['from_pos_y'] = from_y
                data_flow.meta['gui']['editor']['to_pos_x'] = to_x
                data_flow.meta['gui']['editor']['to_pos_y'] = to_y

        if self.selected_outcome is not None and self.last_button_pressed == 1:
            if self.selected_outcome[0] == state.meta['gui']['editor']['outcome_pos']:
                outcome = self.selected_outcome[0][self.selected_outcome[1]]
                cur = self.mouse_move_last_pos
                line_width = min(state.parent.meta['gui']['editor']['width'], state.parent.meta['gui']['editor'][
                    'height']) / 25.0
                self.view.editor.draw_transition(outcome[0], outcome[1], cur[0], cur[1], line_width, [], True,
                                                 depth + 0.6)


    def _find_selection(self, pos_x, pos_y, only_states=False):
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
            (selection, selection_depth) = self._selection_ids_to_model(selected_ids, self.model, 1, None, 0,
                                                                        only_states)
            # print selection, selection_depth
        except Exception as e:
            logger.error("Error while finding selection: {err:s}".format(err=e))
            pass
        return selection

    def _selection_ids_to_model(self, ids, search_state, search_state_depth, selection, selection_depth, only_states):
        """Searches recursively for objects with the given ids

        The method searches recursively and compares all stored ids with the given ones. It finally returns the
        object with the biggest depth (furthest nested)
        :param ids: The ids to search for
        :param search_state: The state to search in
        :param search_state_depth: The depth the search state is in
        :param selection: The currently found object
        :param selection_depth: The depth of the currently found object
        :return: The selected object and its depth
        """
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
                                                                            selection, selection_depth, only_states)

            if len(ids) == 0 or search_state_depth < selection_depth or only_states:
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

    @staticmethod
    def _point_on_line(point, line_start, line_end):
        def distance(a, b):
            return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        ds = 1
        if -ds < (distance(line_start, point) + distance(point, line_end) - distance(line_start, line_end)) < ds:
            return True

        return False