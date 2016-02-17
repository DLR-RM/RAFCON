from gtkmvc import ModelMT

from rafcon.mvc.models.state_element import StateElementModel

from rafcon.statemachine.outcome import Outcome

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

    @ModelMT.observe("outcome", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(OutcomeModel, self).model_changed(model, prop_name, info)
