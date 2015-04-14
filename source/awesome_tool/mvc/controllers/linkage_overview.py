import gtk
from awesome_tool.mvc.controllers.extended_controller import ExtendedController
from gtkmvc import Model

from awesome_tool.mvc.controllers import DataPortListController, ScopedVariableListController, \
    StateOutcomesEditorController

# from awesome_tool.utils import log
# logger = log.get_logger(__name__)


class LinkageOverviewController(ExtendedController, Model):

    def __init__(self, model, view):
        ExtendedController.__init__(self, model, view)

        self.add_controller('inputs_ctrl', DataPortListController(model, view['inputs_view'], "input"))
        self.add_controller('output_ctlr', DataPortListController(model, view['outputs_view'], "output"))
        self.add_controller('scoped_ctrl', ScopedVariableListController(model, view['scopes_view']))
        self.add_controller('outcomes_ctrl', StateOutcomesEditorController(model, view['outcomes_view']))

        view['inputs_view'].show()
        view['outputs_view'].show()
        view['scopes_view'].show()
        view['outcomes_view'].show()