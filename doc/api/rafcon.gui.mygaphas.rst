Gaphas extensions for RAFCON: ``rafcon.gui.mygaphas``
=====================================================

Gaphas is a Python library for state-machine editors using Canvas: https://github.com/gaphor/gaphas

It is used here as library for the graphical editor as replacement for OpenGL. This modules contains all extensions
necessary for RAFCON.

.. contents::
    :backlinks: top

Views
-----

Views are derived from :py:class:`gaphas.item` and are the visual representations for core elements/models.


Perpendicular Line
^^^^^^^^^^^^^^^^^^

The base class for the :py:class:`rafcon.gui.mygaphas.items.connection.ConnectionView`.

.. automodule:: rafcon.gui.mygaphas.items.line
    :members:
    :undoc-members:
    :show-inheritance:

ConnectionViews
^^^^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.items.connection
    :members:
    :undoc-members:
    :show-inheritance:

PortViews
^^^^^^^^^

Each port (income, outcome, data ports) are represented as a view element.

.. automodule:: rafcon.gui.mygaphas.items.ports
    :members:
    :undoc-members:
    :show-inheritance:

StateView and NameView
^^^^^^^^^^^^^^^^^^^^^^

Each :py:class:`rafcon.gui.mygaphas.items.state.StateView` holds a child item
:py:class:`rafcon.gui.mygaphas.items.state.NameView`, as the name of a state can be resized and
repositioned.

.. automodule:: rafcon.gui.mygaphas.items.state
    :members:
    :undoc-members:
    :show-inheritance:


Utility functions
-----------------

Enumerations
^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.utils.enums
    :members:
    :undoc-members:
    :show-inheritance:

Helper methods for drawing operations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.utils.gap_draw_helper
    :members:
    :undoc-members:
    :show-inheritance:

General helper methods
^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.utils.gap_helper
    :members:
    :undoc-members:
    :show-inheritance:

aspect
------

.. automodule:: rafcon.gui.mygaphas.aspect
    :members:
    :undoc-members:
    :show-inheritance:

canvas
------

.. automodule:: rafcon.gui.mygaphas.canvas
    :members:
    :undoc-members:
    :show-inheritance:

connector
---------

.. automodule:: rafcon.gui.mygaphas.connector
    :members:
    :undoc-members:
    :show-inheritance:

constraint
----------

.. automodule:: rafcon.gui.mygaphas.constraint
    :members:
    :undoc-members:
    :show-inheritance:

guide
-----

.. automodule:: rafcon.gui.mygaphas.guide
    :members:
    :undoc-members:
    :show-inheritance:

painter
-------

.. automodule:: rafcon.gui.mygaphas.painter
    :members:
    :undoc-members:
    :show-inheritance:

segment
-------

.. automodule:: rafcon.gui.mygaphas.segment
    :members:
    :undoc-members:
    :show-inheritance:

tools
-----

.. automodule:: rafcon.gui.mygaphas.tools
    :members:
    :undoc-members:
    :show-inheritance:

view
----

.. automodule:: rafcon.gui.mygaphas.view
    :members:
    :undoc-members:
    :show-inheritance:
