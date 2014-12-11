
from gtkmvc import ModelMT
from statemachine.transition import Transition


class TransitionModel(ModelMT):
    """This model class manages a Transition

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a transition).

    :param Transition transition: The transition to be managed
     """

    transition = None

    __observables__ = ("transition",)

    def __init__(self, transition):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters
        
        assert isinstance(transition, Transition)

        self.transition = transition
        return
