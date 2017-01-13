from weakref import ref
from gtkmvc import ModelMT, Signal

from rafcon.gui.models.signals import Notification
from rafcon.gui.models.abstract_state import AbstractStateModel

from rafcon.utils.hashable import Hashable
from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


class StateElementModel(ModelMT, Hashable):
    """This model class serves as base class for all models within a state model (ports, connections)

    Each state element model has a parent, meta and temp data. If observes itself and informs the parent about changes.

    :param rafcon.gui.models.abstract_state.AbstractStateModel parent: The state model of the state element
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
        :rtype: rafcon.gui.models.abstract_state.AbstractState
        """
        if not self._parent:
            return None
        return self._parent()

    @parent.setter
    def parent(self, parent_m):
        """Setter for the parent state model of the state element

        :param rafcon.gui.models.abstract_state.AbstractState parent_m: Parent state model or None
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

    @staticmethod
    def _meta_data_editor_gaphas2opengl(vividict):
        """Convert meta data of editor from gaphas to OpenGL

        This should be implemented in the child classes if needed.

        :param Vividict vividict: meta data for the gaphas editor
        :return: meta data for the OpenGL editor
        :rtype: Vividict
        """
        return vividict

    @staticmethod
    def _meta_data_editor_opengl2gaphas(vividict):
        """Convert meta data of editor from OpenGL to gaphas

        This should be implemented in the child classes if needed.

        :param Vividict vividict: meta data for the OpenGL editor
        :return: meta data for the gaphas editor
        :rtype: Vividict
        """
        return vividict

    def get_meta_data_editor(self, for_gaphas=True):
        """Returns the editor for the specified editor

        This method should be used instead of accessing the meta data of an editor directly. It return sthe meta data
        of the editor available (with priority to the one specified by `for_gaphas`) and converts it if needed.

        :param bool for_gaphas: True (default) if the meta data is required for gaphas, False if for OpenGL
        :return: Meta data for the editor
        :rtype: Vividict
        """
        meta_gaphas = self.meta['gui']['editor_gaphas']
        meta_opengl = self.meta['gui']['editor_opengl']
        assert isinstance(meta_gaphas, Vividict) and isinstance(meta_opengl, Vividict)

        # Use meta data of editor with more keys (typically one of the editors has zero keys)
        from_gaphas = len(meta_gaphas) > len(meta_opengl) or (len(meta_gaphas) == len(meta_opengl) and for_gaphas)

        # Convert meta data if meta data target and origin differ
        if from_gaphas and not for_gaphas:
            self.meta['gui']['editor_opengl'] = self._meta_data_editor_gaphas2opengl(meta_gaphas)
        elif not from_gaphas and for_gaphas:
            self.meta['gui']['editor_gaphas'] = self._meta_data_editor_opengl2gaphas(meta_opengl)

        # only keep meta data for one editor
        del self.meta['gui']['editor_opengl' if for_gaphas else 'editor_gaphas']

        return self.meta['gui']['editor_gaphas'] if for_gaphas else self.meta['gui']['editor_opengl']

    def set_meta_data_editor(self, key, meta_data, from_gaphas=True):
        """Sets the meta data for a specific key of the desired editor

        :param str key: The meta data key, separated by dots if it is nested
        :param meta_data: The value to be set
        :param bool from_gaphas: If the data comes from a gaphas editor
        """
        meta_gui = self.meta['gui']
        meta_gui = meta_gui['editor_gaphas'] if from_gaphas else meta_gui['editor_opengl']

        key_path = key.split('.')
        for key in key_path:
            if key == key_path[-1]:
                meta_gui[key] = meta_data
            else:
                meta_gui = meta_gui[key]
