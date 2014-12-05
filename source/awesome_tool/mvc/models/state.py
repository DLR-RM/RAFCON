
from gtkmvc.model_mt import ListStoreModelMT
from statemachine import State


class StateModel(ListStoreModelMT):
    """This model class manages a State

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param sm.State state: The state to be managed
     """

    state = None

    __observables__ = ("state",)

    _attributes = ['id', 'name']#, 'description', 'is_start', 'is_final']
    _columns = {
        'key':      {'column': 0, 'type': str},
        'value':    {'column': 1, 'type': str},
        'editable': {'column': 2, 'type': bool},
        'name':     {'column': 3, 'type': str}
    }

    def __init__(self, state):
        """Constructor
        """
        types = map(lambda x: x['type'], sorted(self._columns.values()))  # reduce columns to types sorted by column
        ListStoreModelMT.__init__(self, *types)  # pass columns as separate parameters

        assert isinstance(state, State)

        self.state = state
        self.update_attributes()
        return

    def update_attributes(self):
        self.clear()
        for key in self._attributes:
            self.append([key, self.state.__getattribute__(key), 1, key])



    pass