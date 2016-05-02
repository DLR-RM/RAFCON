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

    @ModelMT.observe("transition", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(TransitionModel, self).model_changed(model, prop_name, info)
