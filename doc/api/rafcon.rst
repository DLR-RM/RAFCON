RAFCON API: The ``rafcon`` package
==================================

RAFCON completely separates the GUI from the core, allowing state machines to be created, stored, loaded and started
without having any user interface but the console.

This core of RAFCON resides in the rafcon.core sub-module. It includes classes for all types of states, a
state-machine class and many more.

The rafcon.gui sub-module contains all parts needed for the GUI using GTK. It uses the
Model-View-Controller-architecture (MVC).

Finally, rafcon.utils hold several helping modules, for example for logging.

.. toctree::
    :maxdepth: 3

    rafcon.core
    rafcon.gui
    rafcon.utils
