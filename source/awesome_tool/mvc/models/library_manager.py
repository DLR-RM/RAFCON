
from gtkmvc import ModelMT
from awesome_tool.utils.vividict import Vividict
from awesome_tool.statemachine.library_manager import LibraryManager

from awesome_tool.utils import log
logger = log.get_logger(__name__)


class LibraryManagerModel(ModelMT):
    """This model class manages a library manager

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a library manager).

    :param library_manager: The library_manager to be managed
     """

    library_manager = None

    __observables__ = ("library_manager",)

    def __init__(self, library_manager, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters

        assert isinstance(library_manager, LibraryManager)

        self.library_manager = library_manager

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        # this class is an observer of its own properties:
        self.register_observer(self)
