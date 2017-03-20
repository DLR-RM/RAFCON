Gaphas extensions for RAFCON: ``rafcon.gui.mygaphas``
=====================================================

Gaphas is a Python library for state-machine editors using Canvas: https://github.com/amolenaar/gaphas

It is used her as library for the graphical editor as replacement for OpenGL. This modules contains all extensions
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

ConnectionViews
^^^^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.items.connection


PortViews
^^^^^^^^^

Each port (income, outcome, data ports) are represented as a view element.

.. automodule:: rafcon.gui.mygaphas.items.ports


StateView and NameView
^^^^^^^^^^^^^^^^^^^^^^

Each :py:class:`rafcon.gui.mygaphas.items.state.StateView` holds a child item
:py:class:`rafcon.gui.mygaphas.items.state.NameView`, as the name of a state can be resized and
repositioned.

.. automodule:: rafcon.gui.mygaphas.items.state




Utility functions
-----------------

Enumerations
^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.utils.enums


Helper methods for drawing operations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.utils.gap_draw_helper


General helper methods
^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: rafcon.gui.mygaphas.utils.gap_helper



aspect
------

.. automodule:: rafcon.gui.mygaphas.aspect



canvas
------

.. automodule:: rafcon.gui.mygaphas.canvas



connector
---------

.. automodule:: rafcon.gui.mygaphas.connector



constraint
----------

.. automodule:: rafcon.gui.mygaphas.constraint



guide
-----

.. automodule:: rafcon.gui.mygaphas.guide



painter
-------

.. automodule:: rafcon.gui.mygaphas.painter



segment
-------

.. automodule:: rafcon.gui.mygaphas.segment



tools
-----

.. automodule:: rafcon.gui.mygaphas.tools



view
----

.. automodule:: rafcon.gui.mygaphas.view