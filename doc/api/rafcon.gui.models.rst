MVC Models: ``rafcon.gui.models``
=================================

This package contains all models of the MVC architecture.

The models hold the data for, which are shown in the views. These models typically hold an element of the core, which
is observable. For example, the StateModel holds a reference to a State class object from the core. If the core
element changes, the model recognizes these changes and forwards the notification to the controllers, if they observe
the changed model property.

.. contents::
    :backlinks: top

AbstractStateModel (in abstract_state)
--------------------------------------

.. automodule:: rafcon.gui.models.abstract_state
    :members:
    :undoc-members:
    :show-inheritance:

StateModel (in state)
---------------------

.. automodule:: rafcon.gui.models.state
    :members:
    :undoc-members:
    :show-inheritance:

ContainerStateModel (in container_state)
----------------------------------------

.. automodule:: rafcon.gui.models.container_state
    :members:
    :undoc-members:
    :show-inheritance:

LibraryStateModel (in library_state)
------------------------------------

.. automodule:: rafcon.gui.models.library_state
    :members:
    :undoc-members:
    :show-inheritance:

StateElementModel (in state_element)
------------------------------------

.. automodule:: rafcon.gui.models.state_element
    :members:
    :undoc-members:
    :show-inheritance:

TransitionModel (in transition)
-------------------------------

.. automodule:: rafcon.gui.models.transition
    :members:
    :undoc-members:
    :show-inheritance:

DataFlowModel (in data_flow)
----------------------------

.. automodule:: rafcon.gui.models.data_flow
    :members:
    :undoc-members:
    :show-inheritance:

DataPortModel (in data_port)
----------------------------

.. automodule:: rafcon.gui.models.data_port
    :members:
    :undoc-members:
    :show-inheritance:

ScopedVariableModel (in scoped_variable)
----------------------------------------

.. automodule:: rafcon.gui.models.scoped_variable
    :members:
    :undoc-members:
    :show-inheritance:

Selection (in selection)
------------------------
.. automodule:: rafcon.gui.models.selection
    :members:
    :undoc-members:
    :show-inheritance:

LogicalPortModel (in logical_port)
----------------------------------

.. automodule:: rafcon.gui.models.logical_port
    :members:
    :undoc-members:
    :show-inheritance:

StateMachineModel (in state_machine)
------------------------------------

.. automodule:: rafcon.gui.models.state_machine
    :members:
    :undoc-members:
    :show-inheritance:

ModificationsHistoryModel (in modification_history)
---------------------------------------------------

.. automodule:: rafcon.gui.models.modification_history
    :members:
    :undoc-members:
    :show-inheritance:

AutoBackupModel (in auto_backup)
--------------------------------

.. automodule:: rafcon.gui.models.auto_backup
    :members:
    :undoc-members:
    :show-inheritance:

StateMachineManagerModel (in state_machine_manager)
---------------------------------------------------

.. automodule:: rafcon.gui.models.state_machine_manager
    :members:
    :undoc-members:
    :show-inheritance:

GlobalVariableManagerModel (in global_variable_manager)
-------------------------------------------------------

.. automodule:: rafcon.gui.models.global_variable_manager
    :members:
    :undoc-members:
    :show-inheritance:

LibraryManagerModel (in library_manager)
----------------------------------------

.. automodule:: rafcon.gui.models.library_manager
    :members:
    :undoc-members:
    :show-inheritance:

StateMachineExecutionEngineModel (in state_machine_execution_engine)
--------------------------------------------------------------------

.. automodule:: rafcon.gui.models.state_machine_execution_engine
    :members:
    :undoc-members:
    :show-inheritance:
