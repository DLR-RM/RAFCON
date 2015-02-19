from gtkmvc import ModelMT
import gobject
from gtk import ListStore
import gtk

from statemachine.states.state import State
from table import TableDescriptor, ColumnDescriptor, AttributesRowDescriptor
from utils.vividict import Vividict
from mvc.models.data_port import DataPortModel


class StateModel(ModelMT):
    """This model class manages a State

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param State state: The state to be managed
     """

    state = None
    input_data_ports = []
    output_data_ports = []

    __observables__ = ("state", "input_data_ports", "output_data_ports")

    _table = TableDescriptor()
    _table.add_column(ColumnDescriptor(0, 'key', str))
    _table.add_column(ColumnDescriptor(1, 'value', str))
    _table.add_column(ColumnDescriptor(2, 'name', str))
    _table.add_column(ColumnDescriptor(3, 'editable', bool))
    _table.add_row(AttributesRowDescriptor(0, 'state_id', 'ID', editable=True))
    _table.add_row(AttributesRowDescriptor(1, 'name', 'Name'))

    def __init__(self, state, parent=None, meta=None):
        """Constructor
        """
        ModelMT.__init__(self)
        assert isinstance(state, State)

        self.state = state

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        if isinstance(parent, StateModel):
            self.parent = parent
        else:
            self.parent = None

        self.list_store = ListStore(*self._table.get_column_types())
        self.update_attributes()

        self.register_observer(self)
        self.input_data_ports = []
        self.output_data_ports = []
        self.input_data_port_list_store = ListStore(str, str, str, int)
        self.output_data_port_list_store = ListStore(str, str, str, int)
        self.reload_input_data_port_list_store_and_models()
        self.reload_output_data_port_list_store_and_models()

    def update_attributes(self):
        """Update table model with state model

        Clears all table rows from the model. Then add them back again by iterating over the state attributes and
        adding all items to the table model.
        """
        self.list_store.clear()
        for row in self._table.rows:
            key = row.key
            self.list_store.append([key, self.state.__getattribute__(key), row.name, row.editable])

    def update_row(self, row, value):
        """Update the list store row with the new model

        This method is being called by the controller, when the value within a TreeView has been changed by the user.
        The methods tries to update the state model with the new value and returns the exception object if the valid
        is not valid.

        :param row: The row/path within the ListStore
        :param value: The value set by the user
        :return: True when successful, the exception otherwise
        """
        attr = self.list_store[row][0]
        try:
            self.state.__setattr__(attr, value)
        except ValueError as error:
            return error
        return True

    # @ModelMT.observe("state", after=True)
    # def _model_changed(self, model, name, info):
    #     self.model_changed(model, name, info, self)

    @ModelMT.observe("state", after=True, before=True)
    def model_changed(self, model, name, info):
        if self.parent is not None:
            self.parent.model_changed(model, name, info)

    def get_model_info(self, model):
        model_list = None
        data_list = None
        model_name = ""
        model_class = None
        model_key = None
        if model == "input_data_port":
            model_list = self.input_data_ports
            data_list = self.state.input_data_ports
            model_name = "data_port"
            model_class = DataPortModel
        elif model == "output_data_port":
            model_list = self.output_data_ports
            data_list = self.state.output_data_ports
            model_name = "data_port"
            model_class = DataPortModel
        else:
            raise AttributeError("Wrong model specified!")
        return model_list, data_list, model_name, model_class, model_key

    @ModelMT.observe("state", after=True)
    def update_models(self, model, name, info):
        """ This method is always triggered when the core state changes

            It keeps the following models/model-lists consistent:
            input-data-port models
            output-data-port models
            outcome models
        """
        # TODO: outcomes
        model_list = None
        if "input_data_port" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("input_data_port")
        elif "output_data_port" in info.method_name:
            (model_list, data_list, model_name, model_class, model_key) = self.get_model_info("output_data_port")

        if model_list is not None:
            if "add" in info.method_name:
                self.add_missing_model(model_list, data_list, model_name, model_class, model_key)
            elif "remove" in info.method_name:
                self.remove_additional_model(model_list, data_list, model_name, model_key)

    def reload_input_data_port_list_store(self):
        """Reloads the input data port list store from the data port models
        """
        tmp = ListStore(str, str, str, int)
        for idp_model in self.input_data_ports:
            tmp.append([idp_model.data_port.name, idp_model.data_port.data_type, idp_model.data_port.default_value,
                        idp_model.data_port.data_port_id])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.input_data_port_list_store.clear()
        for elem in tmp:
            self.input_data_port_list_store.append(elem)

    def reload_input_data_port_models(self):
        """Reloads the input data port models directly from the the state
        """
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))

    def reload_input_data_port_list_store_and_models(self):
        """Reloads the input data port list store and models
        """
        self.reload_input_data_port_models()
        self.reload_input_data_port_list_store()

    def reload_output_data_port_list_store(self):
        """Reloads the output data port list store from the data port models
        """
        tmp = ListStore(str, str, str, int)
        for idp_model in self.output_data_ports:
            tmp.append([idp_model.data_port.name, idp_model.data_port.data_type, idp_model.data_port.default_value,
                        idp_model.data_port.data_port_id])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, dataport_compare_method)
        tms.sort_column_changed()
        self.output_data_port_list_store.clear()
        tmp = tms
        for elem in tmp:
            self.output_data_port_list_store.append(elem)

    def reload_output_data_port_models(self):
        """Reloads the output data port models directly from the the state
        """
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))

    def reload_output_data_port_list_store_and_models(self):
        """Reloads the output data port list store and models
        """
        self.reload_output_data_port_models()
        self.reload_output_data_port_list_store()

    def add_missing_model(self, model_list, data_list, model_name, model_class, model_key):
        for data in data_list.itervalues():
            found = False
            for model_item in model_list:
                model = model_item if model_key is None else model_list[model_item]
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list.append(model_class(data, self))
                else:
                    model_list[getattr(data, model_key)] = model_class(data, self)
                return

    def remove_additional_model(self, model_list, data_list, model_name, model_key):
        for model_item in model_list:
            model = model_item if model_key is None else model_list[model_item]
            found = False
            for data in data_list.itervalues():
                if data is getattr(model, model_name):
                    found = True
                    break
            if not found:
                if model_key is None:
                    model_list.remove(model)
                else:
                    del model_list[model_item]
                return


def dataport_compare_method(treemodel, iter1, iter2, user_data=None):
    path1 = treemodel.get_path(iter1)[0]
    path2 = treemodel.get_path(iter2)[0]
    name1 = treemodel[path1][0]
    name2 = treemodel[path2][0]
    name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
    name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
    if name1_as_bits == name2_as_bits:
        return 0
    elif name1_as_bits > name2_as_bits:
        return 1
    else:
        return -1