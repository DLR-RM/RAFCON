.. _plugins_docs:

Plugins
=======

For :ref:`RAFCON`, a plugin concept has been implemented and there
are already some plugins out there. Most notable is the "Monitoring
Plugin", which supports remote access and observation of a state-machine
running somewhere in the network. There is also a plugin template
demonstrating the use of observers to follow changes of RAFCON
observables and how to substitute or monkey patch (if absolutely
necessary) RAFCON classes, functions, etc.. Examples for more advanced
usage of these potentially useful aspects can be found in the plugins
introduced below.

Plugin Interface
----------------

The path of every RAFCON plugin has to be registered in the
environmental variable :envvar:`RAFCON_PLUGIN_PATH`. In the registered path,
the ``__init__.py`` and ``hooks.py`` of respective plugin should be
situated. In the optional ``hooks.py``, the following functions (hooks)
can be implemented.

``pre_init``
""""""""""""

The function is called after all of RAFCON imports occurred and the
RAFCON singletons have been created. In this function, RAFCON classes
can be extended/monkey-patched or completely substituted.
Anyway, it is good to avoid monkey patches or big substitutions of
classes at this point. Especially if you extend, monkey-patch or
substitute a class that is used in a singleton make sure that your
change reaches all parts of RAFCON (by explicitly substituting objects).
An example is given with the Execution-Hooks plugin and the examples in
``share/examples/plugins/templates``.

``post_init``
"""""""""""""

The function is called after the command line parameters have been
processed and everything is initialized (e.g. a state machine which
could be loaded) and the GUI and models are fully initiated (e. g. the
observer of the GUI are registered to the core-object/observables). In
this function, observers should register their observables. A simple
example can be found in the Execution-Hooks plugin, a much more complex
example is given with the Monitoring plugin and examples of example on
observer usages can be found in the plugin template.

``main_window_setup`` (GUI only)
""""""""""""""""""""""""""""""""

The hook is called after the view has been registered, namely at the
very end of the ``register_view`` methods of the
``MainWindowController``. A reference to the main window controller is
passed as argument.

``pre_destruction``
"""""""""""""""""""

When the GUI is running, this hook is called right before the
\`prepare\_destruction\` method of the Menu Bar is being called, thus
before the GUI is being destroyed and closed. When only the core is in
use, the hook is only called id the twisted reactor is running, the call
is made before twisted reactor is stopped.

``post_destruction``
""""""""""""""""""""

When the GUI is running, this method is called after the GTK main loop
has been terminated. Thus the main window has already been destroyed at
this point. When only the core is in use, the hook is called after an
eventual call to stop the twisted reactor, otherwise right before the
program ends.

Available plugins
-----------------

Monitoring Plugin
"""""""""""""""""

This RAFCON plugin enables monitoring RAFCON instances via unsecure
UDP/IP. The RMPM package name is ``rafcon_monitoring_plugin``.
See also: :ref:`Using the monitoring plugin`

DDS Monitoring Plugin
"""""""""""""""""""""

This plugin facilitates monitoring and remote control of RAFCON state
machines via RTI DDS.

Once the plugin has been set up, RAFCON can be started in either server
or client mode. Currently, only one server per DDS domain is supported;
if you would like to control multiple RAFCON instances you will need to
separate them by using different domain IDs.

You can start RAFCON in server mode with or without GUI:

.. code:: bash

    # Start server with GUI
    rafcon --server
    # Start server without GUI
    rafcon_core --remote --server -o <path_to_state_machine> [<path_to_state_machine> ...]

In order to start RAFCON in client mode simply pass the ``--client``
parameter:

.. code:: bash

    rafcon --client

There are no restrictions on the number of clients per domain, so you
can connect as many clients as you wish.

When you start RAFCON in client mode and open a state machine you will
notice a new information bar at the bottom:

.. figure:: _static/Rafcon_dds_monitoring_client.png
   :alt: A screenshot of the dds monitoring rafcon feature
   :width: 90 %
   :align: center

This bar shows the status of the state machine on the remote server.
There are four different states:

-  **Opened locally**
   The state machine is opened locally on the client but not known to
   the remote server. It is therefore not possible to run this state
   machine.
-  **Opened locally and on remote server**
   The state machine is opened both locally and on the remote server. In
   order to start this state machine on the server you will need to make
   it the *active state machine* by clicking on *Make Active*. This
   option is only available if the currently active state machine is not
   running or paused.
-  **Active on remote server**
   The state machine is opened and active on the remote server. You can
   start it by using the common control options.
-  **Running on remote server**
   The state machine is currently running on the remote server.

You can start the clients and server in any order; on startup, they will
automatically retrieve/publish the current state. However, once the
server quits, the information shown on the clients will be outdated
(that is, they will show an active/running state machine even if there
is no server running). Note that there is no "authoritative" client and
the server will process the incoming commands simply in the order they
arrive. Invalid commands will be dismissed.

Execution-Hooks Plugin
""""""""""""""""""""""

This RAFCON plugin enables to use execution hooks on changes in the
execution engine. The RMPM package name is
``rafcon_execution_hooks_plugin``. At the moment, the plugin only
enables this hooks for the state-machine root state.

Plugin Template
"""""""""""""""

The plugin template can be found in ``[RAFCON root path]/share/examples/plugins/templates``.If you put this path into
your :envvar:`RAFCON_PLUGIN_PATH` env var, the plugin will automatically be loaded.
