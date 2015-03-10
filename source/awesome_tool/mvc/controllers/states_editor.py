import pango

import gtk

from mvc.controllers.extended_controller import ExtendedController
from mvc.views.state_editor import StateEditorView, StateEditorEggView, StateEditorLDView
from mvc.controllers.state_editor import StateEditorController, StateEditorEggController, StateEditorLDController
from mvc.models.state_machine_manager import StateMachineManagerModel
from mvc.models.state_machine import Selection
from utils import log
logger = log.get_logger(__name__)


def create_tab_close_button(callback, *additional_parameters):
    closebutton = gtk.Button()
    image = gtk.Image()
    image.set_from_stock(gtk.STOCK_CLOSE, gtk.ICON_SIZE_MENU)
    image_w, image_h = gtk.icon_size_lookup(gtk.ICON_SIZE_SMALL_TOOLBAR)
    closebutton.set_relief(gtk.RELIEF_NONE)
    closebutton.set_focus_on_click(True)
    closebutton.add(image)

    style = gtk.RcStyle()
    style.xthickness = 0
    style.ythickness = 0
    closebutton.modify_style(style)

    closebutton.connect('released', callback, *additional_parameters)

    return closebutton


def limit_tab_label_text(text):
    tab_label_text = text
    if not text or len(text) > 10:
        tab_label_text = text[:5] + '~' + text[-5:]

    return tab_label_text


def generate_tab_label(title):
    label = gtk.Label(title)
    fontdesc = pango.FontDescription("Serif Bold 12")
    label.modify_font(fontdesc)

    return label


def create_tab_header(title, close_callback, *additional_parameters):
    hbox = gtk.HBox()
    hbox.set_size_request(width=-1, height=14)  # safe two pixel
    label = generate_tab_label(title)
    hbox.add(label)

    # add close button
    close_button = create_tab_close_button(close_callback, *additional_parameters)
    hbox.add(close_button)
    hbox.show_all()

    return hbox, label


class StatesEditorController(ExtendedController):

    def __init__(self, model, view, editor_type):

        assert isinstance(model, StateMachineManagerModel)
        ExtendedController.__init__(self, model, view)

        self.__my_selected_state_machine_id = None
        self._selected_state_machine_model = None
        self.editor_type = editor_type

        self.tabs = {}
        self.act_model = None
        self.register()

    @ExtendedController.observe("selected_state_machine_id", assign=True)
    def state_machine_manager_notification(self, model, property, info):
        self.register()

    def register(self):
        """
        Change the state machine that is observed for new selected states to the selected state machine.
        :return:
        """
        # print "states_editor register state_machine"
        # relieve old models
        if self.__my_selected_state_machine_id is not None:  # no old models available
            self.relieve_model(self._selected_state_machine_model.root_state)
            self.relieve_model(self._selected_state_machine_model)
        # set own selected state machine id
        self.__my_selected_state_machine_id = self.model.selected_state_machine_id
        if self.__my_selected_state_machine_id is not None:
            # observe new models
            self._selected_state_machine_model = self.model.state_machines[self.__my_selected_state_machine_id]
            self.observe_model(self._selected_state_machine_model.root_state)
            self.observe_model(self._selected_state_machine_model)  # for selection

    def register_view(self, view):
        # sniffing the graphical viewer selection
        self.view.notebook.connect('switch-page', self.on_switch_page)
        if self._selected_state_machine_model:
            self.add_state_editor(self._selected_state_machine_model.root_state, self.editor_type)

    def add_state_editor(self, state_model, editor_type=None):
        sm_id = self.model.state_machine_manager.get_sm_id_for_state(state_model.state)
        state_identifier = "%s|%s" % (sm_id, state_model.state.get_path())
        # new StateEditor*View
        # new StateEditor*Controller

        if editor_type == 'CommonGrouped':
            state_editor_view = StateEditorView()
            state_editor_ctrl = StateEditorController(state_model, state_editor_view)
        elif editor_type == 'LogicDataGrouped':
            state_editor_view = StateEditorLDView()
            state_editor_ctrl = StateEditorLDController(state_model, state_editor_view)
        else:  # editor_type == 'PortConnectionGrouped':
            state_editor_view = StateEditorEggView()
            state_editor_ctrl = StateEditorEggController(state_model, state_editor_view)
        self.add_controller(state_model.state.state_id, state_editor_ctrl)

        tab_label_text = limit_tab_label_text("%s|%s" % (sm_id, str(state_model.state.name)))
        (evtbox, new_label) = create_tab_header(tab_label_text, self.on_close_clicked,
                                                state_model, 'refused')
        new_label.set_tooltip_text("%s|%s" % (sm_id, str(state_model.state.name)))

        state_editor_view.get_top_widget().title_label = new_label

        idx = self.view.notebook.prepend_page(state_editor_view.get_top_widget(), evtbox)
        page = self.view.notebook.get_nth_page(idx)
        self.view.notebook.set_tab_reorderable(page, True)
        page.show_all()

        state_editor_view.show()
        self.view.notebook.show()
        self.tabs[state_identifier] = {'page': page, 'state_model': state_model,
                                       'ctrl': state_editor_ctrl, 'sm_id': self.__my_selected_state_machine_id,
                                       'view': state_editor_view}

        return idx

    def on_close_clicked(self, event, state_model, result):
        """ Callback for the "close-clicked" emitted by custom TabLabel widget. """
        #print event, state_model, result
        # TODO use sm_id - the sm_id is not found while remove_state
        # TODO           -> work-around is to use only the path to find the page
        # sm_id = self.model.state_machine_manager.get_sm_id_for_state(state_model.state)
        # state_identifier = "%s|%s" % (sm_id, state_model.state.get_path())
        self.tabs["%s|%s" % (None, state_model.state.get_path())]
        page_to_close = None
        for key, items in self.tabs.iteritems():
            if key.split('|')[1] == state_model.state.get_path():  # state_identifier.split('|')[1]:
                page_to_close = items['page']
                break
        #page_to_close = self.tabs[state_identifier]['page']
        # if page_to_close:
        current_idx = self.view.notebook.page_num(page_to_close)

        self.view.notebook.remove_page(current_idx)
        # del self.tabs[state_identifier]['ctrl']
        # del self.tabs[state_identifier]['view']
        # del self.tabs[state_identifier]
        self.remove_controller(state_model.state.state_id)

    def close_all_tabs(self):
        """
        Closes all tabs of the states editor
        :return:
        """
        state_model_list = []
        for identifier, tab in self.tabs.iteritems():
            state_model_list.append(tab['state_model'])
        for state_model in state_model_list:
            self.on_close_clicked(None, state_model, None)

    def on_switch_page(self, notebook, page, page_num, user_param1=None):
        #logger.debug("switch page %s %s" % (page_num, page))
        page = notebook.get_nth_page(page_num)
        for identifier, meta in self.tabs.iteritems():
            if meta['page'] is page:
                model = meta['state_model']
                logger.debug("switch-page %s" % model.state.name)
                if not self._selected_state_machine_model.selection.get_selected_state() == model:
                    self.model.selected_state_machine_id = int(identifier.split('|')[0])
                    self._selected_state_machine_model.selection.set([model])
                    self.act_model = model
                return

    def change_state_editor_selection(self, selected_model):
        state_identifier = "%s|%s" % (self.model.state_machine_manager.get_sm_id_for_state(selected_model.state),
                                       selected_model.state.get_path())
        if self.act_model is None or not self.act_model.state.state_id == selected_model.state.state_id:
            logger.debug("State %s is SELECTED" % selected_model.state.name)

            # print "state_identifier: %s" % state_identifier
            if not state_identifier in self.tabs:
                idx = self.add_state_editor(selected_model, self.editor_type)
                self.view.notebook.set_current_page(idx)

            else:
                page = self.tabs[state_identifier]['page']
                idx = self.view.notebook.page_num(page)
                # print idx
                if not self.view.notebook.get_current_page() == idx:
                    self.view.notebook.set_current_page(idx)
            self.act_model = selected_model

    @ExtendedController.observe("selection", after=True)
    def selection_notification(self, model, property, info):
        selection = info.instance
        assert isinstance(selection, Selection)
        # logger.debug("The viewer should jump as selected to tab in states_editor %s %s %s" % (info.instance, model, property))
        if info.instance.get_num_states() == 1 and len(info.instance) == 1:
            self.change_state_editor_selection(info.instance.get_states()[0])

    @ExtendedController.observe("state", after=True)
    @ExtendedController.observe("states", after=True)
    def notify_state(self, model, prop_name, info):
        # TODO in combination with state type change (remove - add -state) there exist sometimes inconsistencies
        # logger.debug("In States-Editor state %s call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %
        #              ("X", prop_name, info.instance, info.method_name, info))
        # print self.tabs.keys()
        if hasattr(info, "kwargs") and info.method_name == 'state_change':
            if info.kwargs.method_name == 'remove_state':
                logger.debug("remove tab for state %s and search for others to remove" % info.kwargs.args[1])
                #check if state in notebook
                sm_id = self.model.state_machine_manager.get_sm_id_for_state(info.kwargs.args[0])
                identifier = str(sm_id) + '|' + info.kwargs.args[0].get_path() + '/' + info.kwargs.args[1]
                if identifier in self.tabs:
                    state_model = self.tabs[identifier]['state_model']
                    self.on_close_clicked(event=None, state_model=state_model, result=None)
                # search actual not necessary
                #self.remove_search()
            if info.kwargs.method_name == 'name':
                self.check_name()
        if info.method_name in ['__delitem__']:  # , 'remove_state']: taken by state_change
            # self.remove_search()  # this could remove pages of states that are from the other open state machines
            sm_id = self.model.state_machine_manager.get_sm_id_for_state(model.state)
            parent_identifier = str(sm_id) + '|' + model.state.get_path()
            if info.method_name == '__delitem__':
                state_model = self.tabs[parent_identifier + '/' + info.args[0]]['state_model']
            else:  # state
                state_model = self.tabs[parent_identifier + '/' + info.args[1]]['state_model']
            self.on_close_clicked(event=None, state_model=state_model, result=None)
        if info.method_name == 'name':
                self.check_name()

    def remove_search(self):
        to_remove = []
        for path, page_dict in self.tabs.items():
            if page_dict['state_model'].parent and \
                    not page_dict['state_model'].state.state_id in page_dict['state_model'].parent.states:
                logger.debug("remove: ", page_dict['state_model'].state.state_id)
                to_remove.append(page_dict['state_model'])
        for state_model in to_remove:
            self.on_close_clicked(event=None, state_model=state_model, result=None)

    def check_name(self):
        for identifier, page_dict in self.tabs.items():
            identifier_list = identifier.split('|')
            tab_label = identifier_list[0] + '|' + page_dict['state_model'].state.name
            if not page_dict['page'].title_label.get_tooltip_text() == tab_label:
                page_dict['page'].title_label.set_text(limit_tab_label_text(tab_label))
                fontdesc = pango.FontDescription("Serif Bold 12")
                page_dict['page'].title_label.modify_font(fontdesc)
                page_dict['page'].title_label.set_tooltip_text(tab_label)
