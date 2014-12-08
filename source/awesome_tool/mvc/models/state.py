
from gtkmvc import ModelMT
from statemachine import State
from table import TableDescriptor, ColumnDescriptor, AttributesRowDescriptor
from gtk import ListStore


class StateModel(ModelMT):
    """This model class manages a State

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param sm.State state: The state to be managed
     """

    state = None

    __observables__ = ("state",)

    _table = TableDescriptor()
    _table.add_column(ColumnDescriptor(0, 'key', str))
    _table.add_column(ColumnDescriptor(1, 'value', str))
    _table.add_column(ColumnDescriptor(2, 'name', str))
    _table.add_column(ColumnDescriptor(3, 'editable', bool))
    _table.add_row(AttributesRowDescriptor(0, 'id', 'ID', editable=True))
    _table.add_row(AttributesRowDescriptor(1, 'name', 'Name'))

    def __init__(self, state):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters
        assert isinstance(state, State)


        self.state = state
        self.list_store = ListStore(*self._table.get_column_types())
        self.update_attributes()
        return

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

    pass