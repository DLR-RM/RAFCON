"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the GTk GUI

.. moduleauthor:: TODO


"""
from awesome_tool.statemachine.singleton import state_machine_manager
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel

global_focus = None

# This variable holds the global state machine manager model as long as only one StateMachineMangerModel is allowed
state_machine_manager_model = StateMachineManagerModel(state_machine_manager)