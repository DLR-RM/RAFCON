
from gtkmvc import ModelMT
from awesome_tool.statemachine.transition import Transition
from awesome_tool.utils.vividict import Vividict

from awesome_tool.utils import log
logger = log.get_logger(__name__)


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

        self.temp = Vividict()

        # this class is an observer of its own properties:
        self.register_observer(self)

    @ModelMT.observe("transition", before=True, after=True)
    def model_changed(self, model, name, info):

        if self.parent is not None:
            # logger.debug("TRANSITION_MODEL_CHANGED %s %s %s" % (str(model), str(name), str(info)))
            self.parent.model_changed(model, name, info)
