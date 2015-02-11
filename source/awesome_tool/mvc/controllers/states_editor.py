import gtk
from gtkmvc import Controller

from mvc.views.state_editor import StateEditorView, StateEditorEggView, StateEditorLDView
from mvc.controllers.state_editor import StateEditorController, StateEditorEggController, StateEditorLDController
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


class StatesEditorController(Controller):

    def __init__(self, model, view, editor_type):
        Controller.__init__(self, model, view)

        self.editor_type = editor_type

        self.tabs = {}
        self.act_model = None

    def register_view(self, view):
        # sniffing the graphical viewer selection
        self.view.notebook.connect('switch-page', self.on_switch_page)

    def add_state_editor(self, state_model, editor_type=None):
        state_identifier = state_model.state.get_path()
        # new StateEditor*View
        # new StateEditor*Controller
        if editor_type == 'mod':
            state_editor_view = StateEditorView()
            state_editor_ctrl = StateEditorController(state_model, state_editor_view)
        elif editor_type == 'ld':
            state_editor_view = StateEditorLDView()
            state_editor_ctrl = StateEditorLDController(state_model, state_editor_view)
        else:  # editor_type == 'egg':
            state_editor_view = StateEditorEggView()
            state_editor_ctrl = StateEditorEggController(state_model, state_editor_view)

        tab_label_text = state_model.state.name
        if len(state_model.state.name) > 10:
            tab_label_text = state_model.state.name[:10] + '~'
        (evtbox, new_label) = create_tab_header(tab_label_text, self.on_close_clicked,
                                                state_model, 'refused')
        state_editor_view.get_top_widget().title_label = new_label

        idx = self.view.notebook.prepend_page(state_editor_view.get_top_widget(), evtbox)
        page = self.view.notebook.get_nth_page(idx)
        self.view.notebook.set_tab_reorderable(page, True)
        page.show_all()

        state_editor_view.show()
        self.view.notebook.show()
        # print "tab: ", state_identifier
        self.tabs[state_identifier] = {'page': page, 'state_model': state_model, 'ctrl': state_editor_ctrl}

        return idx

    def on_close_clicked(self, event, state_model, result):
        """ Callback for the "close-clicked" emitted by custom TabLabel widget. """
        #print event, state_model, result

        state_identifier = state_model.state.get_path()
        page = self.tabs[state_identifier]['page']
        current_idx = self.view.notebook.page_num(page)

        self.view.notebook.remove_page(current_idx)  # current_idx)  # utils.find_tab(self.notebook, page))
        del self.tabs[state_identifier]

    def on_switch_page(self, notebook, page, page_num, user_param1=None):
        model = None
        # page = notebook.get_current_page()  # slow execution
        # for identifier, meta in self.tabs.iteritems():
        #     print meta, page
        #     if meta['page'] == page:
        #         model = meta['state_model']
        #         break
        # if model:
        #     print "switch-page %s" % model.state.name
        #     self.act_model = model

    def change_state_editor_selection(self, selected_model):
        state_identifier = selected_model.state.get_path()
        if self.act_model is None or not self.act_model.state.state_id == selected_model.state.state_id:
            logger.debug("State %s is SELECTED" % selected_model.state.name)

            from mvc.views.state_editor import StateEditorEggView
            from mvc.controllers.state_editor import StateEditorEggController

            # print "state_identifier: %s" % state_identifier
            if not state_identifier in self.tabs:
                idx = self.add_state_editor(selected_model, self.editor_type)
                self.view.notebook.set_current_page(idx)

            else:
                page = self.tabs[state_identifier]['page']
                idx = self.view.notebook.page_num(page)
                # print idx
                self.view.notebook.set_current_page(idx)
            self.act_model = selected_model
