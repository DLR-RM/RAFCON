from weakref import ref
from gtkmvc import ModelMT, Signal

from rafcon.mvc.models.signals import Notification
from rafcon.mvc.models.abstract_state import AbstractStateModel

from rafcon.utils.hashable import Hashable
from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateElementModel(ModelMT, Hashable):
    """This model class serves as base class for all models within a state model (ports, connections)

    Each state element model has a parent, meta and temp data. If observes itself and informs the parent about changes.

    :param rafcon.mvc.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
    """

    _parent = None
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

    def update_hash(self, obj_hash):
        Hashable.update_hash_from_dict(obj_hash, self.meta)

    @property
    def parent(self):
        """Getter for the parent state model of the state element

        :return: None if parent is not defined, else the model of the parent state
        :rtype: rafcon.mvc.models.abstract_state.AbstractState
        """
        if not self._parent:
            return None
        return self._parent()

    @parent.setter
    def parent(self, parent_m):
        """Setter for the parent state model of the state element

        :param rafcon.mvc.models.abstract_state.AbstractState parent_m: Parent state model or None
        """
        if isinstance(parent_m, AbstractStateModel):
            self._parent = ref(parent_m)
        else:
            self._parent = None

    @property
    def core_element(self):
        """Return the core element represented by this model

        :return: core element of the model
        :rtype: rafcon.core.state_elements.state_element.StateElement
        """
        raise NotImplementedError()

    def prepare_destruction(self):
        """Prepares the model for destruction

        Unregisters the model from observing itself.
        """
        try:
            self.unregister_observer(self)
        except KeyError:  # Might happen if the observer was already unregistered
            pass

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
