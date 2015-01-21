from utils import log
logger = log.get_logger(__name__)

import gtk
from gtkmvc import Controller
from gtkmvc import Observer


class GlobalVariableManagerController(Controller, Observer):

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

        view['new_global_variable_button'].connect('clicked', self.on_new_global_variable_button_clicked)
        view['delete_global_variable_button'].connect('clicked', self.on_delete_global_variable_button_clicked)
        self.new_gv_counter = 0

    #new buttons
    def on_new_global_variable_button_clicked(self, widget, data=None):
        new_global_variable = "a_new_global_variable%s" % self.new_gv_counter
        self.new_gv_counter += 1
        self.model.global_variable_manager.set_variable(new_global_variable, "val")
        pass

    #delete buttons
    def on_delete_global_variable_button_clicked(self, widget, data=None):
        tree_view = self.view["global_variable_tree_view"]
        path = tree_view.get_cursor()[0]
        if path is not None:
            key = self.model.global_variables_list_store[int(path[0])][0][0]
            #print "key to remove: ", key
            self.model.global_variable_manager.delete_global_variable(key)


    @Observer.observe("global_variable_manager", after=True)
    def assign_notification_state(self, model, prop_name, info):
        #print "call_notification - AFTER:\n-%s\n-%s\n-%s\n-%s\n" %\
        #      (prop_name, info.instance, info.method_name, info.result)
        model.update_global_variables_list_store()


    def register_view(self, view):
        """Called when the View was registered
        """

        view.get_top_widget().connect('destroy', gtk.main_quit)

        def cell_text(column, cell_renderer, model, iter, global_variable_manager_model):
            col = column.get_name()
            global_variable = model.get_value(iter, 0)
            if col == 'name_col':
                cell_renderer.set_property('text', global_variable[0])
            elif col == 'value_col':
                cell_renderer.set_property('text', global_variable[1])
            elif col == 'locked_col':
                lock =\
                    global_variable_manager_model.global_variable_manager.variable_locks[global_variable[0]]
                locked = lock.locked()
                cell_renderer.set_property('text', locked)
            else:
                logger.error("Unknown column '{col:s}' in GlobalVariableManagerView".format(col=col))

        #print self.model.global_variables_list_store
        #for elem in self.model.global_variables_list_store:
        #    print "elem is: ", elem

        view["global_variable_tree_view"].set_model(self.model.global_variables_list_store)

        view['name_col'].set_cell_data_func(view['name_text'], cell_text, self.model)
        view['name_text'].set_property("editable", True)
        view['value_col'].set_cell_data_func(view['value_text'], cell_text, self.model)
        view['value_text'].set_property("editable", True)
        view['locked_col'].set_cell_data_func(view['locked_text'], cell_text, self.model)

        view['name_text'].connect("edited", self.on_name_changed)
        view['value_text'].connect("edited", self.on_data_type_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_name_changed(self, widget, path, text):
        #print path
        import copy
        #logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))
        #print self.view.get_top_widget().get_selection().get_selected_rows()
        old_key = self.model.global_variables_list_store[int(path)][0][0]
        old_value = self.model.global_variables_list_store[int(path)][0][1]
        #print "old_key: ", old_key
        #print "old_value: ", old_value
        #delete old global variable
        self.model.global_variable_manager.delete_global_variable(old_key)
        #the text is the new key
        self.model.global_variable_manager.set_variable(text, old_value)
        self.model.update_global_variables_list_store()

    def on_data_type_changed(self, widget, path, text):
        old_key = self.model.global_variables_list_store[int(path)][0][0]
        old_value = self.model.global_variables_list_store[int(path)][0][1]
        self.model.global_variable_manager.set_variable(old_key, text)
        self.model.update_global_variables_list_store()