
from gtkmvc import ModelMT
from statemachine.outcome import Outcome
from utils.vividict import Vividict

from utils import log
logger = log.get_logger(__name__)


class OutcomeModel(ModelMT):
    """This model class manages a Outcome

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a transition).

    :param Outcome outcome: The outcome to be managed
     """

    outcome = None

    __observables__ = ("outcome",)

    def __init__(self, outcome, parent, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters

        assert isinstance(outcome, Outcome)

        self.outcome = outcome
        self.parent = parent

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # this class is an observer of its own properties:
        self.register_observer(self)

    @ModelMT.observe("Outcome", before=True, after=True)
    def model_changed(self, model, name, info):

        if self.parent is not None:
            # logger.debug("OUTCOME_MODEL_CHANGED %s %s %s" % (str(model), str(name), str(info)))
            self.parent.model_changed(model, name, info)
