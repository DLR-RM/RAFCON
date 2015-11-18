from gtkmvc import ModelMT

from rafcon.mvc.models.state_element import StateElementModel

from rafcon.statemachine.data_port import DataPort

from rafcon.utils import log

logger = log.get_logger(__name__)


class DataPortModel(StateElementModel):
    """This model class manages a DataPort

    :param rafcon.statemachine.dat_port.DataPort data_port: The input/output data port to be wrapped
    :param rafcon.mvc.models.abstract_state.AbstractStateModel parent: The state model of the state element
    :param rafcon.utils.vividict.Vividict meta: The meta data of the state element model
     """

    data_port = None

    __observables__ = ("data_port",)

    def __init__(self, data_port, parent, meta=None):
        """Constructor
        """
        super(DataPortModel, self).__init__(parent, meta)

        assert isinstance(data_port, DataPort)
        self.data_port = data_port

    @ModelMT.observe("data_port", before=True, after=True)
    def model_changed(self, model, prop_name, info):
        super(DataPortModel, self).model_changed(model, prop_name, info)
