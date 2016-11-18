from copy import copy, deepcopy
from gtkmvc import ModelMT

from rafcon.mvc.models.state_element import StateElementModel
from rafcon.statemachine.state_elements.outcome import Outcome
from rafcon.utils import log
logger = log.get_logger(__name__)


class OutcomeModel(StateElementModel):
    """This model class manages a Outcome

    :param rafcon.statemachine.outcome.Outcome outcome: The outcome to be wrapped
    :param rafcon.mvc.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    outcome = None

    __observables__ = ("outcome",)

    def __init__(self, outcome, parent, meta=None):
        """Constructor
        """
        super(OutcomeModel, self).__init__(parent, meta)

        assert isinstance(outcome, Outcome)
        self.outcome = outcome

    def __str__(self):
        return "Model of Outcome: {0}".format(self.outcome)

    def __eq__(self, other):
        # logger.info("compare method")
        if isinstance(other, OutcomeModel):
            return self.outcome == other.outcome and self.meta == other.meta
        else:
            return False

    def __copy__(self):
        outcome = copy(self.outcome)
        outcome_m = self.__class__(outcome, parent=None, meta=None)
        outcome_m.meta = deepcopy(self.meta)
        return outcome_m

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @ModelMT.observe("outcome", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(OutcomeModel, self).model_changed(model, prop_name, info)
