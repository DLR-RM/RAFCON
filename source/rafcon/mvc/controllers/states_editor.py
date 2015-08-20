
import gtk

from rafcon.mvc.controllers.extended_controller import ExtendedController
from rafcon.mvc.views.state_editor import StateEditorView
from rafcon.mvc.controllers.state_editor import StateEditorController
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.mvc.models.container_state import StateModel, ContainerStateModel
from rafcon.mvc.selection import Selection
from rafcon.mvc.config import global_gui_config
from rafcon.utils import constants
from rafcon.utils import log
logger = log.get_logger(__name__)


def create_button(toggle, font, font_size, icon_code, release_callback=None, *additional_parameters):
    if toggle:
        button = gtk.ToggleButton()
    else:
        button = gtk.Button()

    button.set_relief(gtk.RELIEF_NONE)
    button.set_focus_on_click(False)
    button.set_size_request(width=18, height=18)

    style = gtk.RcStyle()
    style.xthickness = 0
    style.ythickness = 0
    button.modify_style(style)

    label = gtk.Label()
    label.set_markup("<span font_desc='{0} {1}'>&#x{2};</span>".format(font, font_size, icon_code))
    button.add(label)

    if release_callback:
        button.connect('released', release_callback, *additional_parameters)

    return button


def create_tab_close_button(callback, *additional_parameters):
    close_button = create_button(False, constants.ICON_FONT, constants.FONT_SIZE_SMALL, constants.BUTTON_CLOSE,
                                 callback, *additional_parameters)

    return close_button


def create_sticky_button(callback, *additional_parameters):
    sticky_button = create_button(True, constants.ICON_FONT, constants.FONT_SIZE_SMALL, constants.ICON_STICKY,
                                  callback, *additional_parameters)

    return sticky_button


def create_tab_header(title, close_callback, sticky_callback, *additional_parameters):
    hbox = gtk.HBox()
    hbox.set_size_request(width=-1, height=20)  # safe two pixel

    if global_gui_config.get_config_value('KEEP_ONLY_STICKY_STATES_OPEN', True):
        sticky_button = create_sticky_button(sticky_callback, *additional_parameters)
        hbox.pack_start(sticky_button, expand=False, fill=False, padding=0)
    else:
        sticky_button = None

    label = generate_tab_label(title)
    hbox.pack_start(label, expand=True, fill=True, padding=0)

    # add close button
    close_button = create_tab_close_button(close_callback, *additional_parameters)
    hbox.pack_start(close_button, expand=False, fill=False, padding=0)
    hbox.show_all()

    return hbox, label, sticky_button


def limit_tab_label_text(text):
    tab_label_text = text
    if not text or len(text) > 10:
        tab_label_text = text[:5] + '~' + text[-5:]

    return tab_label_text


def generate_tab_label(title):
    label = gtk.Label(title)

    return label


class StatesEditorController(ExtendedController):
    def __init__(self, model, view, editor_type):

        assert isinstance(model, StateMachineManagerModel)
        ExtendedController.__init__(self, model, view)

        self.__my_selected_state_machine_id = None
        self.__selected_state_machine_model = None
        self.editor_type = editor_type

        # TODO: Workaround used for tab-close on middle click
        # Workaround used for tab-close on middle click
        # Event is fired when the user clicks on the tab with the middle mouse button
        view.notebook.connect("tab_close_event", self.on_close_clicked)

        self.tabs = {}
        self.closed_tabs = {}
        self.current_state_m = None
        self.__buffered_root_state = None  # needed to handle exchange of root_state
        self.register_current_state_machine()

    def get_state_identifier(self, state_m):
        state_machine_id = state_m.state.get_sm_for_state().state_machine_id
        state_path = state_m.state.get_path()
        state_identifier = "{0}|{1}".format(state_machine_id, state_path)
        return state_identifier

    def get_state_tab_name(self, state_m):
        state_machine_id = state_m.state.get_sm_for_state().state_machine_id
        state_name = state_m.state.name
        tab_name = "{0}|{1}".format(state_machine_id, state_name)
        return tab_name

    def get_current_state_m(self):
        """Returns the state model of the currently open tab
        """
        page_id = self.view.notebook.get_current_page()
        if page_id == -1:
            return None
        page = self.view.notebook.get_nth_page(page_id)
        state_identifier = self.get_state_identifier_for_page(page)
        return self.tabs[state_identifier]['state_m']

    def on_close_clicked(self, widget, page_num):
        page_to_close = widget.get_nth_page(page_num)
        self.close_page(self.get_state_identifier_for_page(page_to_close), delete=False)

    @ExtendedController.observe("root_state", assign=True)
    def root_state_changed(self, model, property, info):
        # logger.warn("\n\nroot_state changed %s\n\n" % info)
        old_root_state_m = info['old']

        # TODO commented lines can be deleted with next clean up and function satisfies
        # logger.debug("tabs are:")
        # for tab in self.tabs.itervalues():
        #     logger.debug("%s %s" % (tab['state_m'], tab['state_m'].state.get_path()))
        # logger.debug("closed_tabs are:")
        # for tab in self.closed_tabs.itervalues():
        #     logger.debug("%s %s" % (tab['controller'].model, tab['controller'].model.state.get_path()))

        # close all tabs related child models and root-state-model
        # TODO check if some models are the same in the new model - but only if widgets update if parent has changed
        def close_all_tabs_of_related_state_models_recursively(parent_state_m):
            # logger.debug("run child removes \n%s \n%s" % (parent_state_m, parent_state_m.state))
            if isinstance(parent_state_m, ContainerStateModel):
                # logger.debug("run child removes %s \n%s \n%s" % (parent_state_m, parent_state_m.states, parent_state_m.state))
                # logger.debug("instance %s %s %s" % (parent_state_m, ContainerStateModel, isinstance(parent_state_m, ContainerStateModel)))
                for child_state_m in parent_state_m.states.values():
                    # logger.debug("try to remove: %s %s" % (child_state_m, child_state_m.state.get_path()))
                    close_all_tabs_of_related_state_models_recursively(child_state_m)
            state_identifier = self.get_state_identifier(parent_state_m)
            self.close_page(state_identifier, delete=True)

        close_all_tabs_of_related_state_models_recursively(old_root_state_m)

        # TODO commented lines can be deleted with next clean up and function satisfies
        # logger.debug("final tabs are:")
        # for tab in self.tabs.itervalues():
        #     logger.debug("%s %s" % (tab['state_m'], tab['state_m'].state.get_path()))
        # logger.debug("closed_tabs are:")
        # for tab in self.closed_tabs.itervalues():
        #     logger.debug("%s %s" % (tab['controller'].model, tab['controller'].model.state.get_path()))

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        """Check for changed open state machine

        Register the model of the new state machine, to get notifications about changes in the selection. In
        addition, open the selected state of the new state machine.
        """
        self.register_current_state_machine()

        selection = self.__selected_state_machine_model.selection
        if selection.get_num_states() == 1 and len(selection) == 1:
            self.activate_state_tab(selection.get_states()[0])

    @ExtendedController.observe("state_machines", after=True)
    def state_machines_notification(self, model, prop_name, info):
        """Check for closed state machine and close according states
        """
        if info['method_name'] == '__delitem__':
            states_to_be_removed = []
            for state_identifier, tab_info in self.tabs.iteritems():
                if tab_info['sm_id'] not in self.model.state_machines:
                    states_to_be_removed.append(state_identifier)

            for state_identifier in states_to_be_removed:
                self.close_page(state_identifier, delete=True)

    def register_current_state_machine(self):
        """Change the state machine that is observed for new selected states to the selected state machine.
        """
        # relieve old models
        if self.__my_selected_state_machine_id is not None:  # no old models available
            self.relieve_model(self.__buffered_root_state)
            self.relieve_model(self.__selected_state_machine_model)
        # set own selected state machine id
        self.__my_selected_state_machine_id = self.model.selected_state_machine_id
        if self.__my_selected_state_machine_id is not None:
            # observe new models
            self.__selected_state_machine_model = self.model.state_machines[self.__my_selected_state_machine_id]
            self.__buffered_root_state = self.__selected_state_machine_model.root_state
            self.observe_model(self.__selected_state_machine_model.root_state)
            self.observe_model(self.__selected_state_machine_model)  # for selection

    def register_view(self, view):
        self.view.notebook.connect('switch-page', self.on_switch_page)
        if self.__selected_state_machine_model:
            self.add_state_editor(self.__selected_state_machine_model.root_state, self.editor_type)

    def register_actions(self, shortcut_manager):
        """Register callback methods for triggered actions

        :param rafcon.mvc.shortcut_manager.ShortcutManager shortcut_manager:
        """
        shortcut_manager.add_callback_for_action('rename', self.rename_selected_state)

    def add_state_editor(self, state_m, editor_type=None):
        state_identifier = self.get_state_identifier(state_m)

        if state_identifier in self.closed_tabs:
            state_editor_ctrl = self.closed_tabs[state_identifier]['controller']
            state_editor_view = state_editor_ctrl.view
        else:
            state_editor_view = StateEditorView()
            state_editor_ctrl = StateEditorController(state_m, state_editor_view)

        tab_label_text = self.get_state_tab_name(state_m)
        tab_label_text_trimmed = limit_tab_label_text(tab_label_text)

        (tab, inner_label, sticky_button) = create_tab_header(tab_label_text_trimmed, self.on_tab_close_clicked,
                                                              self.on_toggle_sticky_clicked, state_m)
        inner_label.set_tooltip_text(tab_label_text)

        state_editor_view.get_top_widget().title_label = inner_label
        state_editor_view.get_top_widget().sticky_button = sticky_button

        page_content = state_editor_view.get_top_widget()
        page_id = self.view.notebook.prepend_page(page_content, tab)
        page = self.view.notebook.get_nth_page(page_id)
        self.view.notebook.set_tab_reorderable(page, True)
        page.show_all()

        self.view.notebook.show()
        self.tabs[state_identifier] = {'page': page, 'state_m': state_m,
                                       'controller': state_editor_ctrl, 'sm_id': self.__my_selected_state_machine_id,
                                       'is_sticky': False}
        return page_id

    def close_page(self, state_identifier, delete=True):
        """Closes the desired page

        The page belonging to the state with the specified state_identifier is closed. If the deletion flag is set to
        False, the controller of the page is stored for later usage.
        :param state_identifier: Identifier of the page's state
        :param delete: Whether to delete the controller (deletion is necessary if teh state is deleted)
        """
        # delete old controller references
        if delete and state_identifier in self.closed_tabs:
            del self.closed_tabs[state_identifier]

        # check for open page of state
        if state_identifier in self.tabs:
            page_to_close = self.tabs[state_identifier]['page']
            current_page_id = self.view.notebook.page_num(page_to_close)
            self.view.notebook.remove_page(current_page_id)
            if not delete:
                controller = self.tabs[state_identifier]['controller']
                self.closed_tabs[state_identifier] = {'controller': controller}
            del self.tabs[state_identifier]

    def find_page_of_state_m(self, state_m):
        """Return the identifier and page of a given state model

        :param state_m: The state model to be searched
        :return: page containing the state and the state_identifier
        """
        # TODO use sm_id - the sm_id is not found while remove_state
        # TODO           -> work-around is to use only the path to find the page
        for identifier, page_info in self.tabs.iteritems():
            if page_info['state_m'] is state_m:
                searched_page = page_info['page']
                state_identifier = identifier
                return searched_page, state_identifier
        return None, None

    def on_tab_close_clicked(self, event, state_m):
        [page, state_identifier] = self.find_page_of_state_m(state_m)
        if page:
            self.close_page(state_identifier, delete=False)

    def on_toggle_sticky_clicked(self, event, state_m):
        """Callback for the "toggle-sticky-check-button" emitted by custom TabLabel widget.
        """
        [page, state_identifier] = self.find_page_of_state_m(state_m)
        if not page:
            return
        self.tabs[state_identifier]['is_sticky'] = not self.tabs[state_identifier]['is_sticky']
        page.sticky_button.set_active(self.tabs[state_identifier]['is_sticky'])

    def close_all_pages(self):
        """Closes all tabs of the states editor
        """
        states_to_be_closed = []
        for state_identifier in self.tabs:
            states_to_be_closed.append(state_identifier)
        for state_identifier in states_to_be_closed:
            self.close_page(state_identifier, delete=False)

    def on_switch_page(self, notebook, page_pointer, page_num, user_param1=None):
        """Update state selection when the active tab was changed
        """
        page = notebook.get_nth_page(page_num)

        # find state of selected tab
        for tab_info in self.tabs.values():
            if tab_info['page'] is page:
                state_m = tab_info['state_m']
                sm_id = state_m.state.get_sm_for_state().state_machine_id
                selected_state_m = self.__selected_state_machine_model.selection.get_selected_state()

                # If the state of the selected tab is not in the selection, set it there
                if selected_state_m is not state_m and sm_id in self.model.state_machine_manager.state_machines:
                    self.model.selected_state_machine_id = sm_id
                    self.__selected_state_machine_model.selection.set(state_m)
                return

    def activate_state_tab(self, state_m):
        """Opens the tab for the specified state model

        The tab with the given state model is opened or set to foreground.
        :param state_m: The desired state model (the selected state)
        """

        # The current shown state differs from the desired one
        current_state_m = self.get_current_state_m()
        if current_state_m is not state_m:
            state_identifier = self.get_state_identifier(state_m)

            # The desired state is not open, yet
            if state_identifier not in self.tabs:
                # add tab for desired state
                page_id = self.add_state_editor(state_m, self.editor_type)
                self.view.notebook.set_current_page(page_id)

            # bring tab for desired state into foreground
            else:
                page = self.tabs[state_identifier]['page']
                page_id = self.view.notebook.page_num(page)
                self.view.notebook.set_current_page(page_id)

        self.keep_only_sticked_and_selected_tabs()

    def keep_only_sticked_and_selected_tabs(self):
        """Close all tabs, except the currently active one and all sticked ones
        """
        # Only if the user didn't deactivate this behaviour
        if not global_gui_config.get_config_value('KEEP_ONLY_STICKY_STATES_OPEN', True):
            return

        page_id = self.view.notebook.get_current_page()
        # No tabs are open
        if page_id == -1:
            return

        page = self.view.notebook.get_nth_page(page_id)
        current_state_identifier = self.get_state_identifier_for_page(page)

        states_to_be_closed = []
        # Iterate over all tabs
        for state_identifier, tab_info in self.tabs.iteritems():
            # If the tab is currently open, keep it open
            if current_state_identifier == state_identifier:
                continue
            # If the tab is sticky, keep it open
            if tab_info['is_sticky']:
                continue
            # Otherwise close it
            states_to_be_closed.append(state_identifier)

        for state_identifier in states_to_be_closed:
            self.close_page(state_identifier, delete=False)


    @ExtendedController.observe("selection", after=True)
    def selection_notification(self, model, property, info):
        """If a single state is selected, open the corresponding tab
        """
        selection = info.instance
        assert isinstance(selection, Selection)
        if selection.get_num_states() == 1 and len(selection) == 1:
            self.activate_state_tab(selection.get_states()[0])

    @ExtendedController.observe("state", after=True)
    @ExtendedController.observe("states", after=True)
    def notify_state_removal(self, model, prop_name, info):
        """Close tabs of states that are being removed

        'after' is used here in order to receive deletion events of children first. In addition, only successful
        deletion events are handled. This has the drawback that the model of removed states are no longer existing in
        the parent state model. Therefore, we use the helper method close_state_of_parent, which looks at all open
        tabs as well as closed tabs and the ids of their states.
        """
        def close_state_of_parent(parent_state_m, state_id):

            for tab_info in self.tabs.itervalues():
                state_m = tab_info['state_m']
                # The state id is only unique within the parent
                # logger.debug("tabs: %s %s %s %s" % (state_m.state.state_id, state_id, state_m.parent, parent_state_m))
                if state_m.state.state_id == state_id and state_m.parent is parent_state_m:
                    state_identifier = self.get_state_identifier(state_m)
                    self.close_page(state_identifier, delete=True)
                    return True

            for tab_info in self.closed_tabs.itervalues():
                state_m = tab_info['controller'].model
                # The state id is only unique within the parent
                # logger.debug("closed_tabs: %s %s %s %s" % (state_m.state.state_id, state_id, state_m.parent, parent_state_m))
                if state_m.state.state_id == state_id and state_m.parent is parent_state_m:
                        # state_identifier in self.closed_tabs or state_identifier in self.tabs:
                    state_identifier = self.get_state_identifier(state_m)
                    self.close_page(state_identifier, delete=True)
                    return True
            return False

        # A child state of a root-state child is affected
        if hasattr(info, "kwargs") and info.method_name == 'state_change':
            if info.kwargs.method_name == 'remove_state':
                # logger.warn("child child remove is triggered %s" % info)
                state_id = info.kwargs.args[1]
                parent_state_m = info.kwargs.model
                # logger.debug("remove %s %s %s" % (parent_state_m, parent_state_m.state.state_id, state_id))
                close_state_of_parent(parent_state_m, state_id)
                # logger.debug("tabs are:")
                # for tab in self.tabs.itervalues():
                #     logger.debug("%s %s" % (tab['state_m'], tab['state_m'].state.get_path()))
                # logger.debug("closed_tabs are:")
                # for tab in self.closed_tabs.itervalues():
                #     logger.debug("%s %s" % (tab['controller'].model, tab['controller'].model.state.get_path()))
        # A root-state child is affected
        # -> does the same as the states-model-list observation below IS A BACKUP AT THE MOMENT
        #   -> do we prefer observation of core changes or model changes?
        elif info.method_name == 'remove_state':
            # logger.warn("remove is triggered %s" % info)
            state_id = info.args[1]
            parent_state_m = info.model  # should be root_state
            # logger.debug("remove %s %s %s" % (parent_state_m, parent_state_m.state.state_id, state_id))
            close_state_of_parent(parent_state_m, state_id)
        # for the case that a StateModel gets removed from states-list of root-state-model
        elif info.method_name in ['__delitem__']:
            # logger.warn("delitem is triggered %s" % info)
            state_id = info.args[0]
            parent_state_m = info.model  # is root-state-model
            close_state_of_parent(parent_state_m, state_id)

    @ExtendedController.observe("state", after=True)
    @ExtendedController.observe("states", after=True)
    def notify_state_name_change(self, model, prop_name, info):
        """Checks whether the name of s state was changed and changes the tab label accordingly
        """
        # TODO in combination with state type change (remove - add -state) there exist sometimes inconsistencies
        affected_model = None
        # A child state is affected
        if hasattr(info, "kwargs") and info.method_name == 'state_change':
            if info.kwargs.method_name == 'name':
                affected_model = info.kwargs.model
        # The root state is affected
        elif info.method_name == 'name':
            affected_model = model
        if isinstance(affected_model, StateModel):
            self.update_tab_label(affected_model)

    def update_tab_label(self, state_m):
        """Update all tab labels
        """
        state_identifier = self.get_state_identifier(state_m)
        if state_identifier not in self.tabs:
            return
        page = self.tabs[state_identifier]['page']
        tab_label_text = self.get_state_tab_name(state_m)
        tab_label_text_trimmed = limit_tab_label_text(tab_label_text)
        page.title_label.set_text(limit_tab_label_text(tab_label_text_trimmed))
        page.title_label.set_tooltip_text(tab_label_text)

    def get_state_identifier_for_page(self, page):
        """Return the state identifier for a given page
        """
        for identifier, page_info in self.tabs.iteritems():
            if page_info["page"] is page:  # reference comparison on purpose
                return identifier

    def rename_selected_state(self, key_value, modifier_mask):
        """Callback method for shortcut action rename

        Searches for a single selected state model and open the according page. Page is created if it is not
        existing. Then the rename method of the state controller is called.
        :param key_value:
        :param modifier_mask:
        """
        selection = self.__selected_state_machine_model.selection
        if selection.get_num_states() == 1 and len(selection) == 1:
            selected_state = selection.get_states()[0]
            self.activate_state_tab(selected_state)
            _, state_identifier = self.find_page_of_state_m(selected_state)
            state_controller = self.tabs[state_identifier]['controller']
            state_controller.rename()