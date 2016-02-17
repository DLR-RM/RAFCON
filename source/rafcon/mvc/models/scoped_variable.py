from gtkmvc import ModelMT

from rafcon.mvc.models.state_element import StateElementModel

from rafcon.statemachine.scope import ScopedVariable

from rafcon.utils import log

logger = log.get_logger(__name__)


class ScopedVariableModel(StateElementModel):
    """This model class manages a ScopedVariable

    :param rafcon.statemachine.scoped_variable.ScopedVariable scoped_variable: The scoped variable to be wrapped
    :param rafcon.mvc.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    scoped_variable = None

    __observables__ = ("scoped_variable",)

    def __init__(self, scoped_variable, parent, meta=None):
        """Constructor
        """
        super(ScopedVariableModel, self).__init__(parent, meta)

        assert isinstance(scoped_variable, ScopedVariable)
        self.scoped_variable = scoped_variable

    @ModelMT.observe("scoped_variable", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(ScopedVariableModel, self).model_changed(model, prop_name, info)