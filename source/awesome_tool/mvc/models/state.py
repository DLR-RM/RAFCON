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
        self.input_data_port_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.output_data_port_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.update_input_data_port_list_store_and_models()
        self.update_output_data_port_list_store_and_models()

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

    def update_input_data_port_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        for input_data_port in self.state.input_data_ports.itervalues():
            tmp.append([input_data_port])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.input_data_port_list_store.clear()
        for elem in tmp:
            self.input_data_port_list_store.append(elem)

    def update_input_data_port_models(self):
        self.input_data_ports = []
        for input_data_port in self.state.input_data_ports.itervalues():
            self.input_data_ports.append(DataPortModel(input_data_port, self))

    def update_input_data_port_list_store_and_models(self):
        self.update_input_data_port_list_store()
        self.update_input_data_port_models()

    def update_output_data_port_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        for output_data_port in self.state.output_data_ports.itervalues():
            tmp.append([output_data_port])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, dataport_compare_method)
        tms.sort_column_changed()
        tmp = tms
        self.output_data_port_list_store.clear()
        for elem in tmp:
            self.output_data_port_list_store.append(elem)

    def update_output_data_port_models(self):
        self.output_data_ports = []
        for output_data_port in self.state.output_data_ports.itervalues():
            self.output_data_ports.append(DataPortModel(output_data_port, self))

    def update_output_data_port_list_store_and_models(self):
        self.update_output_data_port_list_store()
        self.update_output_data_port_models()


def dataport_compare_method(treemodel, iter1, iter2, user_data=None):
    path1 = treemodel.get_path(iter1)[0]
    path2 = treemodel.get_path(iter2)[0]
    name1 = treemodel[path1][0].name
    name2 = treemodel[path2][0].name
    name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
    name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
    if name1_as_bits == name2_as_bits:
        return 0
    elif name1_as_bits > name2_as_bits:
        return 1
    else:
        return -1