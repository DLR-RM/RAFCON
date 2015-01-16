
from gtkmvc import ModelMT
from statemachine.transition import Transition
from utils.vividict import Vividict


class TransitionModel(ModelMT):
    """This model class manages a Transition

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a transition).

    :param Transition transition: The transition to be managed
     """

    transition = None

    __observables__ = ("transition",)

    def __init__(self, transition, parent, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters

        assert isinstance(transition, Transition)

        self.transition = transition
        self.parent = parent

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # this class is an observer of its own properties:
        self.register_observer(self)

    @ModelMT.observe("state", before=True, after=True)
    def model_changed(self, model, name, info):
        if self.parent is not None:
            self.parent.model_changed(model, name, info)
