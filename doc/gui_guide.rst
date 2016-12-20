GUI Guide
=========

At this point a common guide for the `RAFCON <home.rst>`__-GUI will be
generated. The page contains advises of how to use the widgets
effectively and will be updated with new features of widgets and their
usage.

Left Bar
--------

Library Tree
""""""""""""

The Library tree shows all library mounted by the LIBRARY\_PATH defined
in your configuration file, see
`RAFCON/Configuration <configuration.rst>`__.Sub-library paths unfold
and fold by double-click and a library directly is open by double-click
it. A right-click on the row of a library element opens a menu with the
options "Add as Library", "Add as template", "Open" and "Open and run".
"Add as template" paste a fully editable copy of respective library into
actual selected state (as long as selected state is a ContainerState).

The Library Tree can be reloaded by pressing the "Refresh"- or the
"Refresh Libraries"-Button in the Menu-Bar or Tool-Bar, above.

State Machine Tree
""""""""""""""""""

The State Machine Tree (or "State Tree") shows all states that are in
the actual selected state-machine, its state\_id and the type of the
state. A state is selected if respective row becomes selected and the
row of a state becomes selected if the state becomes selected by another
widget. A selected state can be removed by pressing the delete-shortcut
or a state can be added into a selected state/row by pressing the
add-shortcut. See `RAFCON/Configuration <configuration.rst>`__ to
define those shortcuts.

Global Variable Manager
"""""""""""""""""""""""

The Global Varaible Widget shows all existing global variables, their
names, values and if they are locked. The name and value can be edited
by clicking into those and "shift-tab", "tab", and the "arrow-keys" can
be used to move throw the editable columns and variables. Press Enter or
move by using "shift-tab" or "tab" to confirm a change of a specific
element. A selected global variable can be removed by pressing the
delete-shortcut or a state can be added into a selected variable/row by
pressing the add-shortcut. See
`RAFCON/Configuration <configuration.rst>`__ to define those
shortcuts.

Modification History
""""""""""""""""""""

.. figure:: assets/EditHistory.jpg
   :width: 25 %
   :alt: Screenshot showing the branching modification history
   :align: right

The history is implemented as a tree of actions. Therefore a trail history view (list)
and a branch history view (tree) are possible. The actual used
representation is the branch history view, see figure.

In the top (below the label "HISTORY") the edit history widget has three
buttons for "Undo/Redo" of actions and "Reset" of the history. The reset
deletes all history tree elements recorded so far. "Undo" and "Redo"
also can be performed by using the shortcuts strg-z and strg-y as usual.

The view also can be modified by the disabling of the branch-history
view (trail-history-view becomes active) using the check-button labeled
"B" or by only enabling automatic folding of the not used branches by
check-button "F".

In the figure it can be seen that the trail history element with number
16 is selected (blue/grey). Every number/id is one step of the state
machine edit process that has been recorded. Action with the number 16
used the method "add transition" and connects two states with the IDs
ZRMXSG and EOSVJL. Elements 17-18 are undone and thereby are labeled
gray. All elements in the trail history from 0 to 16 are in use, so
labeled white. All elements on a previous used branch but not active
branch are also labeled grey, see Nr 11-12 and 3-4.

By selecting a specific element in the list/tree the version after this
action is recovered. To jump to the initial situation of the state
machine when the record of edit history started the user has to click
element 0 which is labeled by action "None".

''' Additional Upcoming Features '''

-  history that is stored to be used after re-opening a state machine

Execution History
"""""""""""""""""

Description in progress ...

Center Pane
-----------

Graphical State Machine Editor
""""""""""""""""""""""""""""""

There are two different graphical editors for editing a state machine.
Originally, the editor used OpenGL and was rather ugly. This is still
the default editor. It is currently being replaced by an editor using
the Python library "Gaphas". The `configuration
documentation <configuration.rst>`__ explains how
to switch between the two editors.

The Gaphas graphical editor is more advanced than the OpenGL one and
also much more pleasant to look at. Only some minor issues currently
prevent it from becoming the default editor. Most features should be
intuitive to use, there are however some shortcuts, which one should be
aware of:

-  Zoom: Ctrl + Scroll wheel or Ctrl + Middle mouse button + Move cursor
   up/down
-  Panning: Middle mouse button
-  Move ports (along border of state): Shift + click and move cursor
-  Move name of state: Ctrl + click and move cursor
-  Resize state with content: Ctrl + click on corner handles
-  Operations on the selected state: Right mouse button (opens context
   menu)
-  Add new Execution State to selected state: Ctrl + A
-  Add new Hierarchy State to selected state: Ctrl + Shift + A

You can also drag'n'drop states into the editor. This is possible from
the four "+ \*S" buttons below the editor and from the libraries widget.

Debug Console
"""""""""""""

The Debug Console can be found below the Grafical Editor. All messages
will be displayed in it, whereas the type of the displayed messages can
be selected with the checkboxes on top of the console. As like the other
widgets, the Debug Console is can be unpinned by clicking the symbol in
the upper right corner. A right-click into the console opens a menu
providing the options to ``"Copy"`` selected output, ``"Select All"``
output or ``"Clear Logging View"``.

Right Bar (States Editor)
-------------------------

The right sidebar shows the "States Editor". It can show several tabs,
but by default, only the selected state is shown. However, you can
*stick* or *pin* a state tab by clicking on the needle icon within the
tab.

The number within the tab shows the state machine id belonging to the
state.

Every "State Editor" consists of the three widgets described below: The
State Overview, State content (with widgets for the Source Editor, Ports
and Connection) and State Description/Summary.

State Overview
""""""""""""""

The State Overview can be found directly under the "STATE EDITOR"
headline. It provides the name of the selected state, which can be
edited by clicking on it, as like the fixed ID of it. Additionally, the
State Overview contains a dropdown menu, where the type of the state can
be changed, and a checkbox which marks a state as start state. (There is
the possibility to pin selected states at the state editor by clicking
on the pin symbol next to the blue highlighted state name, which allows
an userfriendly switching between states.)

Source Editor
"""""""""""""

The Source Editor is the first tab of the notebook in the middle. It is
a numbered editor with the buttons "Apply" to store and "Cancel" to
discard changes.

Outcomes and Transitions
""""""""""""""""""""""""

By clicking the middle tab of the center notebook, the sub-widgets
Outcomes and Transitions can be reached. In the Outcomes widget the
outcomes of the selected state is listed. It consists the "ID" and the
"Name" of the output, the "To-State" it leeds to and if the "To-State"
is a hierachy state the "To-Outcome" of the "To-State". Next to the
obligatory IDs "0", "-1" and "-2", it is possible to append owen
outcomes by clicking the "Add" button. A click on the "Remove" button
will delete the selected outcome.

The Transitions sub-widget lists the transitions between the selected
state and the "Source State" with "Source Outcome" as like the "Target
State" with "Target Outcome". Todo: Internal/External With the buttons
"Add" and "Remove", additional transitions can be added and selected
ones can be deleted.

Data Ports and Data Flows
"""""""""""""""""""""""""

Data Ports and Data Flows sub-widgets can be shown by clicking the last
tab of the middle notebook. Within the Data Ports sub-widget it is
possible to change between "Input Ports" and "Output Ports". The
currently selected one is highlighted in blue. Input and output ports
works like function parameters. They consists of a "Name", a "Data Type"
and a "Default Value" if desired. A click on the button "New" appends a
new port which can be edited while the button "Delete" removes the
selected port.

In "Source State" and "Source Port" column of Data Ports the sources of
the input ports is shown, while "Target State" and "Target Port" shows
the targets of the output ports. With the buttons "Add" and "Remove",
additional rows can be inserted and edited as like selected ones removed

Data Linkage
""""""""""""

Description in progress ...

Logical Linkage
"""""""""""""""

Description in progress ...

Linkage Overview
""""""""""""""""

Description in progress ...

State Description
"""""""""""""""""

The State Description sub-widget can be reached by clicking the second
tab of the lower notebook. It is an editor, where comments or a
description can be placed.
