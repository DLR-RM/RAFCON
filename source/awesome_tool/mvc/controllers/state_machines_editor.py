import gtk

from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from awesome_tool.mvc.views.graphical_editor import GraphicalEditorView
from awesome_tool.mvc.controllers.graphical_editor import GraphicalEditorController
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.mvc.models.state_machine import StateMachineModel, StateMachine
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.utils import log
logger = log.get_logger(__name__)

GAPHAS_AVAILABLE = True
try:
    from awesome_tool.mvc.views.graphical_editor_gaphas import GraphicalEditorView as GraphicalEditorGaphasView
    from awesome_tool.mvc.controllers.graphical_editor_gaphas import GraphicalEditorController as \
        GraphicalEditorGaphasController
except Exception:
    GAPHAS_AVAILABLE = False

import awesome_tool.statemachine.singleton
from awesome_tool.utils import constants, helper
from awesome_tool.mvc.config import global_gui_config


def create_tab_close_button(callback, *additional_parameters):
    close_button = gtk.Button()
    close_button.set_size_request(width=18, height=18)
    close_label = gtk.Label()
    close_label.set_markup('<span font_desc="%s %s">&#x%s;</span>' % (constants.ICON_FONT, constants.FONT_SIZE_SMALL,
                                                                      constants.BUTTON_CLOSE))
    close_button.set_relief(gtk.RELIEF_NONE)
    close_button.set_focus_on_click(True)
    close_button.add(close_label)

    style = gtk.RcStyle()
    style.xthickness = 0
    style.ythickness = 0
    close_button.modify_style(style)

    close_button.connect('released', callback, *additional_parameters)

    return close_button


def create_tab_header(title, close_callback, *additional_parameters):
    hbox = gtk.HBox()
    hbox.set_size_request(width=-1, height=20)  # safe two pixel
    label = gtk.Label(title)
    hbox.add(label)

    # add close button
    close_button = create_tab_close_button(close_callback, *additional_parameters)
    hbox.add(close_button)
    hbox.show_all()

    return hbox


def compose_tab_title(state_machine_id, root_state_name, appendix=''):
    title = "{0}|{1} {2}".format(state_machine_id, root_state_name, appendix)
    title.strip()
    return title


def get_state_machine_id(state_machine_m):
    return state_machine_m.state_machine.state_machine_id


def add_state_machine(widget, event=None):
    """Create a new state-machine when the user clicks on the '+' next to the tabs
    """
    logger.debug("Creating new state-machine...")
    root_state = HierarchyState("new root state")
    state_machine = StateMachine(root_state)
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)


class StateMachinesEditorController(ExtendedController):

    def __init__(self, sm_manager_model, view, states_editor_ctrl):
        ExtendedController.__init__(self, sm_manager_model, view, spurious=True)

        assert isinstance(sm_manager_model, StateMachineManagerModel)
        self.add_controller('states_editor_ctrl', states_editor_ctrl)

        self.tabs = {}

    def register_view(self, view):
        self.view['notebook'].connect("add_state_machine", add_state_machine)
        self.view['notebook'].connect("close_state_machine", self.close_state_machine)
        self.view['notebook'].connect('switch-page', self.on_switch_page)

        # Add all already open state machines
        for state_machine in self.model.state_machines.itervalues():
            self.add_graphical_state_machine_editor(state_machine)

    def register_actions(self, shortcut_manager):
        shortcut_manager.add_callback_for_action('close', self.on_close_shortcut)

        # Call register_action of parent in order to register actions for child controllers
        super(StateMachinesEditorController, self).register_actions(shortcut_manager)

    def close_state_machine(self, widget, page_number):
        """Triggered when the close button in the tab is clicked
        """
        page = widget.get_nth_page(page_number)
        for tab_info in self.tabs.itervalues():
            if tab_info['page'] is page:
                state_machine_m = tab_info['state_machine_m']
                self.on_close_clicked(None, state_machine_m, None, force=False)
                return

    def on_close_shortcut(self, *args):
        """Close active state machine (triggered by shortcut)
        """
        state_machine_m = self.model.get_selected_state_machine_model()
        if state_machine_m is None:
            return
        self.on_close_clicked(None, state_machine_m, None, force=False)

    def on_switch_page(self, notebook, page_pointer, page_num):
        # From documentation: Note the page parameter is a GPointer and not usable within PyGTK. Use the page_num
        # parameter to retrieve the new current page using the get_nth_page() method.
        page = notebook.get_nth_page(page_num)
        for tab_info in self.tabs.itervalues():
            if tab_info['page'] is page:
                state_machine_m = tab_info['state_machine_m']
                new_sm_id = get_state_machine_id(state_machine_m)
                # set active state machine id
                awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = new_sm_id
                if self.model.selected_state_machine_id != new_sm_id:
                    self.model.selected_state_machine_id = new_sm_id
                return

    def get_page_id(self, state_machine_id):
        page = self.tabs[state_machine_id]['page']
        page_id = self.view.notebook.page_num(page)
        return page_id

    def add_graphical_state_machine_editor(self, state_machine_m):
        """Add to for new state machine

        If a new state machine was added, a new tab is created with a graphical editor for this state machine.
        :param state_machine_m: The enw state machine model
        """

        assert isinstance(state_machine_m, StateMachineModel)

        sm_id = get_state_machine_id(state_machine_m)
        logger.debug("Create new graphical editor for state machine with id %s" % str(sm_id))

        if global_gui_config.get_config_value('GAPHAS_EDITOR', False) and GAPHAS_AVAILABLE:
            graphical_editor_view = GraphicalEditorGaphasView()
            graphical_editor_ctrl = GraphicalEditorGaphasController(state_machine_m, graphical_editor_view)
        else:
            graphical_editor_view = GraphicalEditorView()
            graphical_editor_ctrl = GraphicalEditorController(state_machine_m, graphical_editor_view)

        self.add_controller(sm_id, graphical_editor_ctrl)

        unsaved_changed = '*' if state_machine_m.state_machine.marked_dirty else ''
        tab_title = compose_tab_title(sm_id, state_machine_m.root_state.state.name, unsaved_changed)
        tab_label = create_tab_header(tab_title, self.on_close_clicked, state_machine_m, 'refused')

        page = graphical_editor_view['main_frame']
        self.view.notebook.append_page(page, tab_label)
        self.view.notebook.set_tab_reorderable(page, True)
        page.show_all()

        self.tabs[sm_id] = {'page': page,
                            'state_machine_m': state_machine_m}

        graphical_editor_view.show()
        self.view.notebook.show()

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def notification_selected_sm_changed(self, model, prop_name, info):
        """If a new state machine is selected, make sure the tab is open
        """
        selected_state_machine_id = self.model.selected_state_machine_id
        if selected_state_machine_id is None:
            return

        page_id = self.get_page_id(selected_state_machine_id)
        if not self.view.notebook.get_current_page() == page_id:
            self.view.notebook.set_current_page(page_id)

    @ExtendedController.observe("state_machines", after=True)
    def model_changed(self, model, prop_name, info):
        # Check for new state machines
        for sm_id, sm in self.model.state_machine_manager.state_machines.iteritems():
            if sm_id not in self.tabs:
                self.add_graphical_state_machine_editor(self.model.state_machines[sm_id])

        # Check for removed state machines
        state_machines_to_be_deleted = []
        for sm_id in self.tabs:
            if sm_id not in self.model.state_machine_manager.state_machines:
                state_machines_to_be_deleted.append(self.tabs[sm_id]['state_machine_m'])
        for state_machine_m in state_machines_to_be_deleted:
            self.remove_state_machine(state_machine_m)

    @ExtendedController.observe("state_machine_mark_dirty", assign=True)
    def sm_marked_dirty(self, model, prop_name, info):
        sm_id = self.model.state_machine_mark_dirty
        if sm_id in self.model.state_machine_manager.state_machines:
            tab_title = compose_tab_title(sm_id, self.tabs[sm_id]["state_machine_m"].root_state.state.name, '*')
            tab_label = \
                create_tab_header(tab_title, self.on_close_clicked, self.tabs[sm_id]["state_machine_m"], 'refused')
            self.view.notebook.set_tab_label(self.tabs[sm_id]["page"], tab_label)

    @ExtendedController.observe("state_machine_un_mark_dirty", assign=True)
    def sm_un_marked_dirty(self, model, prop_name, info):
        sm_id = self.model.state_machine_un_mark_dirty
        if sm_id in self.model.state_machine_manager.state_machines:
            tab_title = compose_tab_title(sm_id, self.tabs[sm_id]["state_machine_m"].root_state.state.name)
            tab_label = \
                create_tab_header(tab_title, self.on_close_clicked, self.tabs[sm_id]["state_machine_m"], 'refused')
            self.view.notebook.set_tab_label(self.tabs[sm_id]["page"], tab_label)

    def on_close_clicked(self, event, state_machine_m, result, force=False):
        """ Callback for the "close-clicked" emitted by custom TabLabel widget. """

        if force:
            self.remove_state_machine(state_machine_m)
        elif state_machine_m.state_machine.marked_dirty:
            sm_id = get_state_machine_id(state_machine_m)
            root_state_name = state_machine_m.root_state.state.name
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message_string = "There are unsaved changed in the state machine. Do you want to close the " \
                             "state machine '{0}' with id {1} anyway?".format(root_state_name, sm_id)
            message.set_markup(message_string)
            message.add_button("Yes", 42)
            message.add_button("No", 43)
            message.connect('response', self.on_close_message_dialog_response_signal, state_machine_m)
            helper.set_button_children_size_request(message)
            message.show()
        else:
            self.remove_state_machine(state_machine_m)

    def on_close_message_dialog_response_signal(self, widget, response_id, state_machine_m):
        if response_id == 42:
            self.remove_state_machine(state_machine_m)
        else:
            logger.debug("Closing of state machine model canceled")
        widget.destroy()

    def remove_state_machine(self, state_machine_m):
        sm_id = get_state_machine_id(state_machine_m)

        self.remove_controller(sm_id)

        # Close tab and remove info
        page_id = self.get_page_id(sm_id)
        self.view.notebook.remove_page(page_id)
        del self.tabs[sm_id]

        self.model.state_machine_manager.remove_state_machine(sm_id)

        # Open tab with next state machine
        sm_keys = self.model.state_machine_manager.state_machines.keys()
        if len(sm_keys) > 0:
            self.model.selected_state_machine_id = \
                self.model.state_machine_manager.state_machines[sm_keys[0]].state_machine_id
        else:
            self.model.selected_state_machine_id = None

        # if state_machine_m.state_machine.marked_dirty:
        #     state_machine_m.state_machine.marked_dirty = False

    def close_all_tabs(self):
        """Closes all tabs of the state machines editor
        """
        state_machine_m_list = []
        for tab in self.tabs.itervalues():
            state_machine_m_list.append(tab['state_machine_m'])
        for state_machine_m in state_machine_m_list:
            self.on_close_clicked(None, state_machine_m, None, force=True)
