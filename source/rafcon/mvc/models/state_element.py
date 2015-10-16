from gtkmvc import ModelMT, Signal

from rafcon.mvc.models.abstract_state import Notification

from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateElementModel(ModelMT):
    """This model class serves as base class for all models within a state model (ports, connections)

    Each state element model has a parent, meta and temp data. If observes itself and informs the parent about changes.

    :param rafcon.mvc.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
    """

    parent = None
    meta = None
    meta_signal = Signal()
    temp = None

    __observables__ = ("meta_signal",)

    def __init__(self, parent, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)

        self.parent = parent

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()
        self.meta_signal = Signal()

        self.temp = Vividict()

        # this class is an observer of its own properties:
        self.register_observer(self)

    def model_changed(self, model, prop_name, info):
        """This method notifies the parent state about changes made to the state element
        """
        if self.parent is not None:
            self.parent.model_changed(model, prop_name, info)

    @ModelMT.observe("meta_signal", signal=True)
    def meta_changed(self, model, prop_name, info):
        """This method notifies the parent state about changes made to the meta data
        """
        if self.parent is not None:
            msg = info.arg
            # Add information about notification to the signal message
            notification = Notification(model, prop_name, info)
            msg = msg._replace(notification=notification)
            info.arg = msg
            self.parent.meta_changed(model, prop_name, info)
