from gtkmvc import ModelMT
from statemachine.scope import ScopedVariable
from utils.vividict import Vividict


class ScopedVariableModel(ModelMT):
    """This model class manages a ScopedVariable

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a scoped variable).

    :param ScopedVariable scoped_variable: The scoped variable to be managed
     """

    scoped_variable = None

    __observables__ = ("scoped_variable",)

    def __init__(self, scoped_variable, parent, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)

        assert isinstance(scoped_variable, ScopedVariable)
        self.register_observer(self)

        self.scoped_variable = scoped_variable
        self.parent = parent


        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @ModelMT.observe("scoped_variable", after=True)
    def model_changed(self, model, prop_name, info):
        if not self.parent is None:
            self.parent.update_scoped_variables_list_store()
