
from utils import log
logger = log.get_logger(__name__)

import gtk
from gtkmvc import Controller
from gtkmvc.adapters import UserClassAdapter
from mvc.controllers.input_data_port_list import InputDataPortListController, InputDataPortListObserver



class StateDataPortEditorController(Controller):

    #model will be a container state model
    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)
        self.input_data_port_list_controller = InputDataPortListController(model, view.input_port_list_view)
        self.input_data_port_observer = InputDataPortListObserver()
        self.input_data_port_observer.observe_model(model)

    def register_view(self, view):

        view['state_dataport_editor'].connect('destroy', gtk.main_quit)

