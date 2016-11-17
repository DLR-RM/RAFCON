from copy import copy, deepcopy
from gtkmvc import ModelMT

from rafcon.mvc.models.state_element import StateElementModel
from rafcon.statemachine.state_elements.transition import Transition
from rafcon.utils import log
logger = log.get_logger(__name__)


class TransitionModel(StateElementModel):
    """This model class manages a Transition

    :param rafcon.statemachine.transition.Transition transition: The transition to be wrapped
    :param rafcon.mvc.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    transition = None

    __observables__ = ("transition",)

    def __init__(self, transition, parent, meta=None):
        """Constructor
        """
        super(TransitionModel, self).__init__(parent, meta)

        assert isinstance(transition, Transition)
        self.transition = transition

    def __str__(self):
        return "Model of Transition: {0}".format(self.transition)

    def __eq__(self, other):
        # logger.info("compare method")
        if isinstance(other, TransitionModel):
            return self.transition == other.transition and self.meta == other.meta
        else:
            return False

    def __copy__(self):
        transition = copy(self.transition)
        transition_m = self.__class__(transition, parent=None, meta=None)
        transition.meta = deepcopy(self.meta)
        return transition_m

    __deepcopy__ = __copy__

    @property
    def core_element(self):
        return self.transition

    @ModelMT.observe("transition", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(TransitionModel, self).model_changed(model, prop_name, info)
