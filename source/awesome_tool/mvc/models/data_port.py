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
        self.register_observer(self)

        assert isinstance(data_port, DataPort)

        self.data_port = data_port
        self.parent = parent

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

    @ModelMT.observe("data_port", after=True)
    def model_changed(self, model, prop_name, info):
        if not self.parent is None:
            # print "UPDATE PORT PARENT", model.parent.state.state_id, self.parent.state.state_id
            # if model.data_port.data_port_id in self.parent.state.input_data_ports:
            #     self.parent.reload_input_data_port_list_store()
            # if model.data_port.data_port_id in self.parent.state.output_data_ports:
            #     self.parent.reload_output_data_port_list_store()
            self.parent.reload_input_data_port_list_store()
            self.parent.reload_output_data_port_list_store()

            self.parent.model_changed(model, prop_name, info)