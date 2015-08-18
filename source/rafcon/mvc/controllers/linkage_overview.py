import gtk
from rafcon.mvc.controllers.extended_controller import ExtendedController
from gtkmvc import Model

from rafcon.mvc.controllers import DataPortListController, ScopedVariableListController, \
    StateOutcomesEditorController

# from rafcon.utils import log
# logger = log.get_logger(__name__)


class LinkageOverviewController(ExtendedController, Model):

    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)

        self.add_controller('inputs_ctrl', DataPortListController(model, view['inputs_view'], "input"))
        self.add_controller('output_ctlr', DataPortListController(model, view['outputs_view'], "output"))
        self.add_controller('scoped_ctrl', ScopedVariableListController(model, view['scope_view']))
        self.add_controller('outcomes_ctrl', StateOutcomesEditorController(model, view['outcomes_view']))

        view['inputs_view'].show()
        view['outputs_view'].show()
        view['scope_view'].show()
        view['outcomes_view'].show()