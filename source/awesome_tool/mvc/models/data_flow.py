
from gtkmvc import ModelMT
from statemachine import DataFlow


class DataFlowModel(ModelMT):
    """This model class manages a DataFlow

    The model class is part of the MVC architecture. It holds the data to be shown (in this case a data flow).

    :param DataFlow data_flow: The data flow to be managed
     """

    data_flow = None

    __observables__ = ("data_flow",)

    def __init__(self, data_flow):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters
        
        assert isinstance(data_flow, DataFlow)

        self.data_flow = data_flow
        return
