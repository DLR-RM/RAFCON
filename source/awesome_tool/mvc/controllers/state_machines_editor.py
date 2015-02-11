import gtk
from gtkmvc import Controller, Observer

from mvc.views.graphical_editor import GraphicalEditorView
from mvc.controllers.graphical_editor import GraphicalEditorController
from mvc.models.state_machine import StateMachineModel
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


import pango


def create_tab_header(title, close_callback, *additional_parameters):
    hbox = gtk.HBox()
    hbox.set_size_request(width=-1, height=14)  # safe two pixel
    label = gtk.Label(title)
    hbox.add(label)

    fontdesc = pango.FontDescription("Serif Bold 12")
    label.modify_font(fontdesc)

    # add close button
    close_button = create_tab_close_button(close_callback, *additional_parameters)
    hbox.add(close_button)
    hbox.show_all()

    return hbox, label


class ChildObserver(Observer):

    def __init__(self, model, funct_handle_list):
        Observer.__init__(self, model)
        self.func_handle_list = funct_handle_list
        # self.observe(self.notify, "state", after=True)
        self.method_list = ["add", "remove", "clear", "append", "set"]

    @Observer.observe('selection', after=True)
    def notification(self, model, prop_name, info):
        # limit methods
        if info.method_name in self.method_list:
            # print "parent data_flowList call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
            #       (prop_name, info.instance, info.method_name, info.result)
            for func_handle in self.func_handle_list:
                func_handle(model, prop_name, info)


class StateMachinesEditorController(Controller):

    def __init__(self, model, view, state_machine_tree_ctrl, states_editor_ctrl):
        Controller.__init__(self, model, view)
        # print view

        self.state_machine_tree_ctrl = state_machine_tree_ctrl
        self.states_editor_ctrl = states_editor_ctrl

        self.tabs = {}
        self.act_model = None
        self._view = view

    def register_view(self, view):

        if type(self.model.state_machines) is dict:
            for sm_id, sm in self.model.state_machines.iteritems():
                self.add_graphical_state_machine_editor(sm)
        else:
            logger.debug('ManagerShould have dict as state_machines attr')

        #self.state_machine_tree_ctrl.view.connect('cursor-changed', self.on_tree_view_state_double_clicked)

    def add_graphical_state_machine_editor(self, state_machine_model):

        assert isinstance(state_machine_model, StateMachineModel)

        observer = ChildObserver(state_machine_model, [self.selection_notification])

        sm_identifier = state_machine_model.root_state.state.get_path()

        graphical_editor_view = GraphicalEditorView()

        graphical_editor_ctrl = GraphicalEditorController(state_machine_model, graphical_editor_view)
        (event_box, new_label) = create_tab_header(state_machine_model.root_state.state.name,
                                                   self.on_close_clicked,
                                                   state_machine_model, 'refused')
        graphical_editor_view.get_top_widget().title_label = new_label

        idx = self._view.notebook.append_page(graphical_editor_view['main_frame'], event_box)
        page = self._view.notebook.get_nth_page(idx)
        page.show_all()

        self.tabs[sm_identifier] = {'page': page,
                                    'state_model': state_machine_model,
                                    'ctrl': graphical_editor_ctrl,
                                    'connected': False,
                                    'observer': observer,}

        # def on_expose_event(widget, event, sm_identifier):
        #     if not self.tabs[sm_identifier]['connected']:
        #         logger.debug("connect button-release-event with graphical viewer of " + sm_identifier)
        #         self.tabs[sm_identifier]['ctrl'].view.editor.connect('button-release-event',
        #                                                              self.on_graphical_state_double_clicked,
        #                                                              sm_identifier)
        #         self.tabs[sm_identifier]['connected'] = True
        #
        # graphical_editor_view.editor.connect('expose-event', on_expose_event, sm_identifier)
        # #graphical_editor_view.editor.connect('expose-event', self.on_graphical_state_double_clicked, sm_identifier)

        graphical_editor_view.show()
        self._view.notebook.show()

        return idx

    def on_close_clicked(self, event, state_model, result):
        """ Callback for the "close-clicked" emitted by custom TabLabel widget. """
        # print event, state_model, result

        state_identifier = state_model.state.name + '.' + state_model.state.state_id
        page = self.tabs[state_identifier]['page']
        current_idx = self.view.notebook.page_num(page)

        self.view.notebook.remove_page(current_idx)  # current_idx)  # utils.find_tab(self.notebook, page))
        del self.tabs[state_identifier]

    def on_tree_view_state_double_clicked(self, model, property, info):
        (model, row) = self.state_machine_tree_ctrl.view.get_selection().get_selected()
        #if row is not None:
        #    self.states_editor_ctrl.change_state_editor_selection(model[row][3])

    # def on_graphical_state_double_clicked(self, widget, signal_id, sm_identifier):
    #     print widget, signal_id, sm_identifier
    #
    #     def find_selected(state_model):
    #         # print "test state %s " % state_model.state.name
    #         if state_model.meta['gui']['selected']:
    #             return state_model
    #         else:
    #             if hasattr(state_model, 'states'):
    #                 result = None
    #                 # print state_model.states
    #                 for state_id, state_model in state_model.states.iteritems():
    #                     result = find_selected(state_model)
    #                     if result:
    #                         break
    #                 return result
    #             else:
    #                 return None
    #
    #     selected_model = find_selected(self.tabs[sm_identifier]['state_model'])
    #     if selected_model:
    #         self.states_editor_ctrl.change_state_editor_selection(selected_model)
    #     # else:
    #     #     print "no state selected"

    def selection_notification(self, model, property, info):
        # logger.debug("The viewer should jump as selected to tab in states_editor %s %s %s" % (info.instance, model, property))
        if info.instance.get_num_states() == 1 and len(info.instance) == 1:
            self.states_editor_ctrl.change_state_editor_selection(info.instance.get_states()[0])
