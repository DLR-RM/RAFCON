from gtkmvc import ModelMT

from rafcon.statemachine.singleton import global_variable_manager

from rafcon.utils.vividict import Vividict
from rafcon.utils import log
logger = log.get_logger(__name__)


class GlobalVariableManagerModel(ModelMT):
    """This Model class manages the global variable manager."""

    global_variable_manager = global_variable_manager

    __observables__ = ("global_variable_manager",)

    def __init__(self, meta=None):
        """Constructor"""
        ModelMT.__init__(self)  # pass columns as separate parameters

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()
