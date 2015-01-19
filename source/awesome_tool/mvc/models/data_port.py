from gtkmvc import ModelMT
from statemachine.states.state import DataPort
from utils.vividict import Vividict


class DataPortModel(ModelMT):
    """This model class manages a DataPort

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a data port).

    :param DataPort data_port: The data port to be managed
     """

    data_port = None

    __observables__ = ("data_port",)

    def __init__(self, data_port, parent, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters

        assert isinstance(data_port, DataPort)

        self.data_port = data_port
        self.parent = parent

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()
