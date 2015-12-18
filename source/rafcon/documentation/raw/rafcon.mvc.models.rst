MVC Models (rafcon.mvc.models)
====================================

This package contains all models of the MVC architecture.

The models hold the data for, which are shown in the views. These models typically hold an element of the core, which
is observable. For example, the StateModel holds a reference to a State class object from the core. If the core
element changes, the model recognizes these changes and forwards the notification to the controllers, if they observe
the changed model property.

.. contents::
    :backlinks: top

AbstractStateModel (in abstract_state)
--------------------------------------

.. automodule:: rafcon.mvc.models.abstract_state

StateModel (in state)
---------------------

.. automodule:: rafcon.mvc.models.state


ContainerStateModel (in container_state)
----------------------------------------

.. automodule:: rafcon.mvc.models.container_state


StateElementModel (in state_element)
------------------------------------

.. automodule:: rafcon.mvc.models.state_element


TransitionModel (in transition)
-------------------------------

.. automodule:: rafcon.mvc.models.transition


DataFlowModel (in data_flow)
----------------------------

.. automodule:: rafcon.mvc.models.data_flow


DataPortModel (in data_port)
----------------------------

.. automodule:: rafcon.mvc.models.data_port


ScopedVariableModel (in scoped_variable)
----------------------------------------

.. automodule:: rafcon.mvc.models.scoped_variable


OutcomeModel (in outcome)
-------------------------

.. automodule:: rafcon.mvc.models.outcome


StateMachineModel (in state_machine)
------------------------------------

.. automodule:: rafcon.mvc.models.state_machine


StateMachineManagerModel (in state_machine_manager)
---------------------------------------------------

.. automodule:: rafcon.mvc.models.state_machine_manager


GlobalVariableManagerModel (in global_variable_manager)
-------------------------------------------------------

.. automodule:: rafcon.mvc.models.global_variable_manager


LibraryManagerModel (in library_manager)
----------------------------------------

.. automodule:: rafcon.mvc.models.library_manager
