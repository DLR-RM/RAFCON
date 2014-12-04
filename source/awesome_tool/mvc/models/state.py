
from gtkmvc import ModelMT
from statemachine import State


class StateModel(ModelMT):
    """This model class manages a State

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a state).

    :param sm.State state: The state to be managed
     """

    state = None

    __observables__ = ("state",)

    def __init__(self, state):
        """Constructor
        """
        ModelMT.__init__(self)

        assert isinstance(state, State)

        self.state = state
        return


    pass