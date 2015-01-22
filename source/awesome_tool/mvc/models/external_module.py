from gtkmvc import ModelMT
from statemachine.external_modules.external_module import ExternalModule
from utils.vividict import Vividict


class ExternalModuleModel(ModelMT):
    """This model class manages a ExternalModule

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a external module).

    :param ExternalModule external_module: The external module to be managed
     """

    external_module = None

    __observables__ = ("external_module",)

    def __init__(self, external_module, external_module_manager_model, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters
        
        assert isinstance(external_module, ExternalModule)

        self.external_module = external_module
        self.external_module_manager_model = external_module_manager_model

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # this class is an observer of its own properties:
        self.register_observer(self)

    @ModelMT.observe("external_module", after=True)
    def model_changed(self, model, name, info):
        print "external module changed!"
        if self.external_module_manager_model is not None:
            self.external_module_manager_model.update_external_modules_list_store()