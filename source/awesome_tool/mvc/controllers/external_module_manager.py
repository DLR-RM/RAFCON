import inspect
import gtk
from gtkmvc import Controller
from gtkmvc import Observer

from utils import log
logger = log.get_logger(__name__)
from statemachine.external_modules.external_module import EMStatus


class ExternalModuleManagerController(Controller, Observer):

    def __init__(self, model, view, source_view=None):
        """Constructor
        """
        Controller.__init__(self, model, view)

        view['connect_button'].connect('clicked', self.on_connect_button_clicked)
        view['disconnect_button'].connect('clicked', self.on_disconnect_button_clicked)
        view['start_button'].connect('clicked', self.on_start_button_clicked)
        view['stop_button'].connect('clicked', self.on_stop_button_clicked)
        view['pause_button'].connect('clicked', self.on_pause_button_clicked)
        model.update_external_modules_list_store()
        model.reset_external_module_model()

        self.source_view = source_view



    def on_connect_button_clicked(self, widget, data=None):
        tree_view = self.view["external_modules_tree_view"]
        path = tree_view.get_cursor()[0]
        #print path
        if not path is None:
            external_module_name = self.model.external_modules_tree_store[int(path[0])][0]
            #print "external_module_name: ", external_module_name
            #print "Trigger connect for external module: ", external_module_name
            self.model.external_module_manager.external_modules[external_module_name].connect()

    def on_disconnect_button_clicked(self, widget, data=None):
        tree_view = self.view["external_modules_tree_view"]
        path = tree_view.get_cursor()[0]
        if not path is None:
            external_module_name = self.model.external_modules_tree_store[int(path[0])][0]
            #print "Trigger disconnect for external module: ", external_module_name
            self.model.external_module_manager.external_modules[external_module_name].disconnect()

    def on_start_button_clicked(self, widget, data=None):
        tree_view = self.view["external_modules_tree_view"]
        path = tree_view.get_cursor()[0]
        if not path is None:
            external_module_name = self.model.external_modules_tree_store[int(path[0])][0]
            #print "Trigger disconnect for external module: ", external_module_name
            self.model.external_module_manager.external_modules[external_module_name].start()

    def on_stop_button_clicked(self, widget, data=None):
        tree_view = self.view["external_modules_tree_view"]
        path = tree_view.get_cursor()[0]
        if not path is None:
            external_module_name = self.model.external_modules_tree_store[int(path[0])][0]
            #print "Trigger disconnect for external module: ", external_module_name
            self.model.external_module_manager.external_modules[external_module_name].stop()

    def on_pause_button_clicked(self, widget, data=None):
        tree_view = self.view["external_modules_tree_view"]
        path = tree_view.get_cursor()[0]
        if not path is None:
            external_module_name = self.model.external_modules_tree_store[int(path[0])][0]
            #print "Trigger disconnect for external module: ", external_module_name
            self.model.external_module_manager.external_modules[external_module_name].pause()

    @Observer.observe("external_module_manager", after=True)
    def assign_notification_external_module_manager(self, model, prop_name, info):
        #print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #      (prop_name, info.instance, info.method_name, info.result)
        if info.method_name == "add_external_module":
            model.update_external_modules_list_store()
            model.reset_external_module_model()


    def register_view(self, view):
        """Called when the View was registered
        """

        view.get_top_widget().connect('destroy', gtk.main_quit)

        def cell_text(column, cell_renderer, model, iter):
            col = column.get_name()
            external_module_name = model.get_value(iter, 0)
            external_module_status = model.get_value(iter, 1)
            external_module_action = model.get_value(iter, 2)
            #print "external_module_name", external_module_name
            #print "external_module_status", external_module_status
            #print "external_module_action", external_module_action
            actions_store = gtk.ListStore(str, str)
            actions_store.append([0, "insert"])
            actions_store.append([0, "insert_with_parameters"])
            actions_store.append([1, "help"])
            if col == 'status_col':
                readable_status = self.convert_em_status_to_string(external_module_status)
                #print "readable_status", readable_status
                cell_renderer.set_property('text', readable_status)
            elif col == 'name_col':
                cell_renderer.set_property('text', external_module_name)
            elif col == 'actions_col':
                parent_iter = self.model.external_modules_tree_store.iter_parent(iter)
                if parent_iter is None:
                    # no actions for parents
                    actions_store = gtk.ListStore(str, str)
                    cell_renderer.set_property("editable", False)
                    cell_renderer.set_property('text', " ")
                    cell_renderer.set_property('text-column', 1)
                    cell_renderer.set_property('model', actions_store)
                else:
                    cell_renderer.set_property("editable", True)
                    cell_renderer.set_property('text', "choose action")
                    cell_renderer.set_property('text-column', 1)
                    cell_renderer.set_property('model', actions_store)
            else:
                logger.error("Unknown column '{col:s}' in ExternalModuleManagerView".format(col=col))

        view["external_modules_tree_view"].set_model(self.model.external_modules_tree_store)

        #select = view["external_modules_tree_view"].get_selection()
        #select.connect("changed", self.on_tree_selection_changed)

        view['name_col'].set_cell_data_func(view['name_text'], cell_text)
        view['status_col'].set_cell_data_func(view['status_text'], cell_text)
        view['actions_col'].set_cell_data_func(view['actions_combo'], cell_text)
        view['actions_combo'].connect("edited", self.on_combo_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_combo_changed(self, widget, path, text):
        #print path
        iter = self.model.external_modules_tree_store.get_iter(path)
        selected_method = self.model.external_modules_tree_store[iter][0]
        #print selected_method
        parent_iter = self.model.external_modules_tree_store.iter_parent(iter)
        if parent_iter is None:
            #print "parent iter is None"
            # no method was selected, just a parent external module entry
            return
        em_name = self.model.external_modules_tree_store[parent_iter][0]
        #print em_name
        external_module = self.model.external_module_manager.external_modules[em_name]
        if text == "insert":
            if self.source_view:
                #print "the method %s should be inserted into the source field" % selected_method
                self.source_view.get_buffer().insert_at_cursor(selected_method)
        elif text == "insert_with_parameters":
            if self.source_view:
                args = str(inspect.getargspec(getattr(external_module.external_module_class, selected_method)).args)
                self.source_view.get_buffer().insert_at_cursor(selected_method+args)
        elif text == "help":
            #print "A dialog box should pop up"
            method_docu = inspect.getdoc(getattr(external_module.external_module_class, selected_method))
            message = gtk.MessageDialog(type=gtk.MESSAGE_INFO, buttons=gtk.BUTTONS_NONE, flags=gtk.DIALOG_MODAL)
            message.set_markup(method_docu)
            message.add_button("Close", 42)
            message.connect('response', self.on_message_dialog_response_signal_close)
            message.show()

    def on_message_dialog_response_signal_close(self, widget, response_id):
        #print widget
        #print response_id
        #print "Close button pressed"
        widget.destroy()


    def on_tree_selection_changed(self, selection):
        model, treeiter = selection.get_selected()
        if not treeiter is None:
            #print "You selected: ", model[treeiter][0], " ", model[treeiter][1], " ", model[treeiter][2]
            pass

    def convert_em_status_to_string(self, status):
        if status == "EM_STATUS.CONNECTED":
            return "Connected"
        elif status == "EM_STATUS.DISCONNECTED":
            return "Disconnected"
        elif status == "EM_STATUS.STARTED":
            return "Started"
        elif status == "EM_STATUS.STOPPED":
            return "Stopped"
        elif status == "EM_STATUS.PAUSED":
            return "Paused"
        else:
            return None