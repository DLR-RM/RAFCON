Configuration
=============

:ref:`RAFCON` can be configured using two config files, one for
the core and one for the GUI. The config files are automatically
generated (if not existing) on the first run of RAFCON. It is stored in
your home folder: ``~/.config/rafcon/`` with name ``config.yaml`` and
``gui_config.yaml``, respectively. The path can be changed when running
the ``start.py`` script with argument "-c". The syntax used is
`YAML <wp:YAML>`__.

Core configuration
------------------

.. _core_config_example:

Example
"""""""

A typical config file looks like this:

.. code:: yaml

    TYPE: SM_CONFIG

    LIBRARY_PATHS:
        generic: ${RAFCON_LIB_PATH}/generic
        my_home_libs: ~/my_rafcon_libs
        project_libs: ./libs_relative_to_config

.. _core_config_docs:

Documentation
"""""""""""""

In the following, all possible parameters are described, together with
their default value:

TYPE
  | Type: String-constant
  | Default: ``SM_CONFIG``
  | Specifying the type of configuration. Must be SM\_CONFIG for the
    core config file.

LIBRARY\_PATHS
  | Type: Dictionary with type(key) = String and type(value) = String
  | Default: ``{"generic": "${RAFCON_LIB_PATH}/generic"}``
  | A dictionary holding all libraries with name and path. The key of
    the dictionary is a unique library identifier. This unique
    identifier will be used as library name, shown as root of the
    library hierarchy in the library tree. The value of the dictionary
    is a relative or absolute path on the file system that is searched
    for libraries. Relative paths are assumed to be relative to the
    config file. Environment variables are also allowed.

LIBRARY\_RECOVERY\_MODE
  | Type: boolean
  | Default: ``False``
  | If this flag is activated, state machine with consistency erros concerning their data ports can be loaded.
    Erros are just printed out as warnings. This can be used to fix erroneous state machines.

EXECUTION\_LOG\_ENABLE
  | Type: boolean
  | Default: ``True``
  | Enables the logging of rafcon exeuction histories to the file system. Every time a statemachine is executed, a python shelve is created in the execution log directory, e.g. ``/tmp/rafcon_execution_logs/rafcon_execution_log_99-Bottles-of-Beer_2017-08-31-16-07-17.shelve``. Some helpful utility functions for working with log files through python are in: ``import rafcon.utils.execution_log``. A tiny tiny code snippet which shows how to use the pandas.DataFrame representation to query the outcomes of a state named ‘CheckFinished’ is here: ``https://rmc-github.robotic.dlr.de/common/rafcon/pull/324#issuecomment-2520``

EXECUTION\_LOG\_PATH:
  | Type: String
  | Default: ``"/tmp/"``
  | Sets the target path of the execution logs
  
NO\_PROGRAMMATIC\_CHANGE\_OF\_LIBRARY\_STATES\_PERFORMED
  | Type: boolean
  | Default: ``False``
  | Set this to True if you can make sure that the interface of library states is not programmatically changed anywhere inside your state machines. This will speed up loading of libraries.
  
GUI configuration
-----------------

.. _gui_config_example:

Example
"""""""

A typical config file looks like this:

.. code:: yaml

    TYPE: GUI_CONFIG

    SOURCE_EDITOR_STYLE: rafcon-dark

    GAPHAS_EDITOR: True
    GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE: True
    ENABLE_CACHING: True    # Affects only Gaphas editor

    WAYPOINT_SNAP_ANGLE: 45
    WAYPOINT_SNAP_MAX_DIFF_ANGLE: 10
    WAYPOINT_SNAP_MAX_DIFF_PIXEL: 50

    PORT_SNAP_DISTANCE: 5

    LOGGING_SHOW_DEBUG: False
    LOGGING_SHOW_INFO: True
    LOGGING_SHOW_WARNING: True
    LOGGING_SHOW_ERROR: True
    CONSOLE_FOLLOW_LOGGING: True

    LIBRARY_TREE_PATH_HUMAN_READABLE: False
    SUBSTITUTE_STATE_KEEPS_STATE_NAME: True

    MINIMUM_SIZE_FOR_CONTENT: 30
    MAX_VISIBLE_LIBRARY_HIERARCHY: 2

    USE_ICONS_AS_TAB_LABELS: True

    SHOW_NAMES_ON_DATA_FLOWS: True
    ROTATE_NAMES_ON_CONNECTIONS: False
    HISTORY_ENABLED: True

    KEEP_ONLY_STICKY_STATES_OPEN: True

    AUTO_BACKUP_ENABLED: True
    AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL: False
    AUTO_BACKUP_FORCED_STORAGE_INTERVAL: 120
    AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL: 20
    AUTO_RECOVERY_CHECK: False
    AUTO_RECOVERY_LOCK_ENABLED: False

    SESSION_RESTORE_ENABLED: False

    NUMBER_OF_RECENT_OPENED_STATE_MACHINES_STORED: 20

    AUTO_APPLY_SOURCE_CODE_CHANGES: True

    CHECK_PYTHON_FILES_WITH_PYLINT: False

    DEFAULT_EXTERNAL_EDITOR: gvim
    PREFER_EXTERNAL_EDITOR: False

    RESTORE_UNDOCKED_SIDEBARS: False

    SHORTCUTS:
        abort: Escape
        add: <Control>A
        add_execution_state: <Alt>E
        add_hierarchy_state:
        - <Alt>H
        - <Control><Shift>A
        add_preemptive_state: <Alt>C
        add_barrier_state: <Alt>B
        add_output: <Alt>U
        add_input: <Alt>N
        add_outcome: <Alt>T
        add_scoped_variable: <Alt>V
        apply: <Control><Shift>E
        backward_step: F9
        close: <Control>W
        copy: <Control>C
        cut: <Control>X
        data_flow_mode: <Control><Shift>D
        delete: Delete
        down:
        - <Control>Down
        - <Control><Shift>Down
        fit: <Control>space
        group: <Control>G
        info: <Control>I
        is_start_state: <Control>E
        left:
        - <Control>Left
        - <Control><Shift>Left
        new: <Control>N
        open: <Control>O
        open_external_editor: <Control><Shift>Q
        paste: <Control>V
        pause: F7
        quit: <Control>Q
        redo:
        - <Control>Y
        - <Control><Shift>Z
        reload: <Shift>F5
        rename: F2
        right:
        - <Control>Right
        - <Control><Shift>Right
        run_to_selected: <Control><Shift>R
        save: <Control>S
        save_as: <Control><Shift>S
        save_as_copy: <Control><Shift><Alt>S
        save_state_as: <Control><Alt>S
        substitute_state: <Control><Shift><Alt>S
        show_aborted_preempted: <Control>P
        show_data_flows: <Control>D
        show_data_values: <Control>L
        start: F5
        start_from_selected: <Control>R
        step: F4
        step_mode: F6
        stop: F8
        undo: <Control>Z
        ungroup:
        - <Control><Shift>G
        - <Control>U
        up:
        - <Control>Up
        - <Control><Shift>Up
        fullscreen: F11

.. _gui_config_docs:

Documentation
"""""""""""""

TYPE
  | Type: String-constant
  | Default: ``GUI_CONFIG``
  | Specifying the type of configuration. Must be GUI\_CONFIG for the
    GUI config file.

SOURCE\_EDITOR\_STYLE
  | Type: string
  | Default: ``awesome-style``
  | The gtk source view style used in the script editor. Note: You can
    download different styles at
    `https://wiki.gnome.org/Projects/GtkSourceView/StyleSchemes GTK
    Source View
    Styles <https://wiki.gnome.org/Projects/GtkSourceView/StyleSchemes_GTK_Source_View_Styles>`__.
    The scripts have to be downloaded to
    ~/.local/share/gtksourceview-2.0/styles. "awesome-style" is a style
    created to fit to the design of RAFCON.

GAPHAS\_EDITOR
  | Type: boolean
  | Default: ``True``
  | RAFCON started with a graphical editor using Gaphas. The develment of OpenGL
    has been stopped (except bugfixes) in favor of a new editor using
    GTK cairo and the library Gaphas. The flag decides whether to use
    the old OpenGL editor (False) or the new Gaphas one (True).

GAPHAS\_EDITOR\_AUTO\_FOCUS\_OF\_ROOT\_STATE
  | Type: boolean
  | Default: ``True``
  | If RAFCON is started with Gaphas editor enabled this flag enables an
    initial auto focus of the root state after opening the state machine.
    If you do not like this feature simply disable it (False).

ENABLE\_CACHING:
  | Default: ``True``
  | Affects only Gaphas editor and enables a accelerating caching feature.

WAYPOINT\_SNAP\_ANGLE
  | Default: ``45``
  | Unit: Degree
  | Base angle, to which waypoints are snapped to when moving them with
    the Shift key pressed. For a value of 45, waypoints are snapped to
    e. g. 0°, 45°, 90°, 135°, ... Only used in the old editor (OpenGL).

WAYPOINT\_SNAP\_MAX\_DIFF\_ANGLE
  | Default: ``10``
  | Unit: Degree
  | Max deviation to a snap angle, at which the waypoint is still
    snapped. For a value of 10 with a snap angle of 45, the waypoint is
    snapped if the angle of the actual transition/data flow is 99, but
    not if the angle is 102. Only used in the old editor (OpenGL).

WAYPOINT\_SNAP\_MAX\_DIFF\_PIXEL
  | Default: ``50``
  | Unit: px
  | Max snap point distance to the mouse cursor that is still allowed.
    If the waypoint would be snapped according to snap angle and its
    deviation, but the resulting waypoint is too far away from the mouse
    cursor, snapping is aborted. Only used in the old editor (OpenGL).

PORT\_SNAP\_DISTANCE
  | Default: ``5``
  | Unit: Pixel
  | Maximum distane to a port, at which the moved end of a connection is
    snapped to a port (outcome, input, output, scoped variable). Only
    used in Gaphas editor.

LOGGING\_SHOW\_VERBOSE
  | Type: boolean
  | Default: ``False``
  | The flag decide to activate the VERBOSE log level in the logging console view.

LOGGING\_SHOW\_DEBUG
  | Type: boolean
  | Default: ``False``
  | The flag decide to activate the DEBUG log level in the logging console view.
    
LOGGING\_SHOW\_INFO
  | Type: boolean
  | Default: ``True``
  | The flag decide to activate the INFO log level in the logging console view.
    
LOGGING\_SHOW\_WARNING
  | Type: boolean
  | Default: ``True``
  | The flag decide to activate the WARNING log level in the logging console view.
    
LOGGING\_SHOW\_ERROR
  | Type: boolean
  | Default: ``True``
  | The flag decide to activate the ERROR log level in the logging console view.

CONSOLE\_FOLLOW\_LOGGING
  | Type: boolean
  | Default: ``True``
  | The flag decide to activate the follow mode in the logging console view and to stay on the last printed logger message.

LIBRARY\_TREE\_PATH\_HUMAN\_READABLE
  | Type: boolean
  | Default: ``False``
  | The flag is substituting underscores with spaces in the library
    tree. Thereby it is thought for people who do not like spaces in
    file system paths but don't wanna have underscores in the library
    tree.

SUBSTITUTE\_STATE\_KEEPS\_STATE\_NAME
  | Type: boolean
  | Default: ``True``
  | The flag describes the default behavior of the substitute state action
    concerning the previous state name and the state name after the substitution.
    In the dialogs this can be set adapted for the single operation via a check box.
    If the flag is True the name is taken from the original state.
    If the flag is False the name is taken from the state machine that substitute the original state.

MINIMUM\_SIZE\_FOR\_CONTENT
  | Default: ``30``
  | Unit: Pixel
  | Minimum side length (width and height) for container states to have
    their content (child states, transitions, etc.) shown. Currently
    only used in the old editor (OpenGL).

MAX\_VISIBLE\_LIBRARY\_HIERARCHY
  | Default: ``2``
  | Number of hierarchy levels to be shown within a library state. High
    values cause the GUI to lag.

NO\_FULLY\_RECURSIVE\_LIBRARY\_MODEL
  | Type: boolean
  | Default: ``True``
  | If True, GUI models are only loaded up to the MAX\_VISIBLE\_LIBRARY\_HIERARCHY. Setting this to False will drastically increase the time for loading a state machine.
    
USE\_ICONS\_AS\_TAB\_LABELS
  | Type: boolean
  | Default: ``True``
  | If True, only icons will be shown in the tabs on the left and right
    side. Otherwise also a title text is shown.

SHOW\_NAMES\_ON\_DATA\_FLOWS
  | Type: boolean
  | Default: ``True``
  | If False, data flow labels will not be shown (helpful if there are
    many data flows)

ROTATE\_NAMES\_ON\_CONNECTIONS
  | Type: boolean
  | Default: ``False``
  | If True, connection labels will be parallel to the connection.
    Otherwise, they are horizontally aligned.

SHOW\_CONTENT\_LIBRARY\_NAME\_TRANSPARENCY
  | Type: float
  | Default: ``0.5``
  | Set to a value between 0 and 1. Defines the transparency of the name of a LibraryState in the graphical editor,
    of which the content is shown.

HISTORY\_ENABLED
  | Type: boolean
  | Default: ``True``
  | If True, an edit history will be created, allowing for undo and redo
    operation. Might still be buggy, therefore its optional.

KEEP\_ONLY\_STICKY\_STATES\_OPEN
  | Type: boolean
  | Default: ``True``
  | If True, only the currently selected state and sticky states are
    open in the states editor on the right side. Thus, a new selected
    state closes the old one. If False, all states remain open, if they
    are not actively closed.

AUTO\_BACKUP\_ENABLED
  | Type: boolean
  | Default: ``True``
  | If True, the auto backup is enabled. I False, the auto-backup is
    disabled.

AUTO\_BACKUP\_ONLY\_FIX\_FORCED\_INTERVAL
  | Type: boolean
  | Default: ``False``
  | If True, the auto backup is performed according a fixed time
    interval which is defined by
    ``AUTO_BACKUP_FORCED_STORAGE_INTERVAL``. If False, the auto-backup
    is performed dynamically according
    ``AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL`` and will be forced if a
    modification is made more then ``*_FORCED_STORAGE_INTERVAL`` after
    the last backup to the ``/tmp/``-folder. So in case of dynamic
    backup it is tried to avoid user disturbances by waiting for a
    time-interval ``*_DYNAMIC_STORAGE_INTERVAL`` while this the user has
    not modified the state-machine to trigger the auto-backup while
    still using ``*_FORCED_STORAGE_INTERVAL`` as a hard limit.
AUTO\_BACKUP\_FORCED\_STORAGE\_INTERVAL
  | Default: 120
  | Unit: Seconds
  | Time horizon for forced auto-backup if
    ``AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL`` is False and otherwise the
    it is the fix auto-backup time interval.

AUTO\_BACKUP\_DYNAMIC\_STORAGE\_INTERVAL
  | Default: 20
  | Unit: Seconds
  | Time horizon after which the "dynamic" auto-backup
    (``AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL`` is False) is triggered if
    there was no modification to the state-machine while this interval.

AUTO\_RECOVERY\_CHECK
  | Default: ``False``
  | If True, the auto back module will check for backups of crashed instances or
    badly closed state machines that left a lock file. This comfortable feature
    only can be used if the crashed instances or state machines already were
    created with ``AUTO_RECOVERY_LOCK_ENABLED`` and ``AUTO_BACKUP_ENABLED`` True
    and thereby needed lock-files were set.


AUTO\_RECOVERY\_LOCK\_ENABLED:
  | Default: ``False``
  | If True, the auto backup will put lock-files into the respective backup folder
    to label not correctly/cleanly closed state machines and instances.
    The auto recovery check is searching for these locks.

SESSION\_RESTORE\_ENABLED:
  | Default: ``True``
  | If True the current session is stored into the runtime configuration and restored
    after restarting RAFCON with respective runtime configuration file.

NUMBER\_OF\_RECENT\_OPENED\_STATE\_MACHINES\_STORED:
  | default: 20
  | Maximum number of stored recently opened state machine paths.

RESTORE\_UNDOCKED\_SIDEBARS
  | Default: ``False``
  | If True, RAFCON will restore undocked windows from the last
    RAFCON-instance run.

DEFAULT\_EXTERNAL\_EDITOR
  | Default: Empty
  | Holds the command which is executed before the script.py file by clicking the
    'Open externally' button in the source editor window. The command can be anything 
    you wish and results in a shell command with the following pattern:
    '<DEFAULT\_EXTERNAL\_EDITOR> script.py>'.

PREFER_EXTERNAL_EDITOR
  | Default: ``False``
  | If True, RAFCON will assume that the user always wants to work with a different editor
    than the internal one. If the 'Open externally' button is clicked, the source text is 
    locked the whole time and a 'Reload' buttons reloads the saved file into RAFCON.
    If False, it is recommended to close the externally opend script.py everytime you are
    done editing.

SEMANTIC_DATA_MODE
  | Default: ``False``
  | If True, RAFCON gives the semantic data editor of each state more vertical space.
    The vertical space is taken from the port/connection widget. This is especially useful, when working with semantic data.

SHOW_PATH_NAMES_IN_EXECUTION_HISTORY
  | Default: ``False``
  | If True, RAFCON shows the state paths next to the state names in each execution history entry.

SHORTCUTS
  | Type: dict
  | Default: see example ``gui_config.yaml`` above
  | Defines the shortcuts of the GUI. The key describes the action
    triggered by the shortcut, the value defines the shortcut(s). There
    can be more than one shortcut registered for one action. See `GTK
    Documentation <https://people.gnome.org/~gcampagna/docs/Gtk-3.0/Gtk.accelerator_parse.html>`__
    about for more information about the shortcut parser. Not all
    actions are implemented, yet. Some actions are global within the GUI
    (such as 'save'), some are widget dependent (such as 'add').


Environment variables
---------------------

Next to the configuration files, a number of environment variables exist that allow for further configuration.

:envvar:`RAFCON_LOGGING_CONF`
"""""""""""""""""""""""""""""

See :ref:`Logging configuration`.

:envvar:`RAFCON_LIBRARY_PATH`
"""""""""""""""""""""""""""""

An alternative option to specify your RAFCON libraries, which can e.g. be handy in combination with RMPM. See
:ref:`tutorial_rafcon_library_path`.

:envvar:`RAFCON_PLUGIN_PATH`
""""""""""""""""""""""""""""

Use this variable to specify the RAFCON plugins that are to be loaded. See :ref:`Plugin Interface`.

:envvar:`RAFCON_START_MINIMIZED`
""""""""""""""""""""""""""""""""

If the env variable :envvar:`RAFCON_START_MINIMIZED` is set (i.e., has a value which is not an empty string), RAFCON is
started minimized/iconified. This comes in handy, when the tests are run. You can then continue working, without
RAFCON windows repeatedly being opened and closed in the foreground.


Logging configuration
---------------------

RAFCON uses the default Python ``logging`` package for logging. Starting with version 0.9.7, logging handlers,
filters, formatting and more can be configured using a JSON file. The default configuration can be found in
``source/rafcon/logging.conf``. The configuration can be overwritten with a custom JSON file. To do so, specify the
path to your configuration in the env variable :envvar:`RAFCON_LOGGING_CONF`. For information about the ``logging``
package, please check the `official documentation <https://docs.python.org/2/library/logging.html>`__.

.. _logging_config_example:

Example
"""""""

To not destroy the behavior of RAFCON, the default configuration should be used as basis for your extensions. The
following example shows how to add another logging handler, writing all messages to a file:

.. code:: json

    {
        ...

        "loggers": {
            ...
            "rafcon": {
                ...
                "handlers": ["stdout", "stderr", "loggingView", "file"]
            }
        },

        "handlers": {
            ...
            "file": {
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "default",
                "filename": "/tmp/rafcon.log",
                "maxBytes": 1024,
                "backupCount": 3
            }
        },

        ...
    }


Monitoring plugin configuration
-------------------------------

The config file of the monitoring plugin contains all parameters and
settings for communication. It is additionally needed next to the
``config.yaml`` and the ``gui_config.yaml`` to run the plugin. If it
does not exist, it will be automatically generated by the first start of
the ``start.py`` and stored at ``~/.config/rafcon`` as
``network_config.yaml``. The path of the used config file can be changed
by launching the ``start.py`` script with argument "-nc".

.. _monitoring_plugin_example:

Example
"""""""

The default ``network_config.file`` looks like:

.. code:: yaml

    BURST_NUMBER: 1
    CLIENT_UDP_PORT: 7777
    ENABLED: true
    HASH_LENGTH: 8
    HISTORY_LENGTH: 1000
    MAX_TIME_WAITING_BETWEEN_CONNECTION_TRY_OUTS: 3.0
    MAX_TIME_WAITING_FOR_ACKNOWLEDGEMENTS: 1.0
    SALT_LENGTH: 6
    SERVER: true
    SERVER_IP: 127.0.0.1
    SERVER_UDP_PORT: 9999
    TIME_BETWEEN_BURSTS: 0.01
    TYPE: NETWORK_CONFIG

.. _monitoring_plugin_docs:

Documentation
"""""""""""""

BURST\_NUMBER
  | Type: int
  | Default: ``1``
  | Amount of messages with the same content which shall be send to
    ensure the communication.

CLIENT\_UDP\_PORT
  | Type: int
  | Default: ``7777``
  | Contains the UDP port of the client

ENABLED
  | Type: boolean
  | Default: ``True``

HASH\_LENGHT
  | Type: int
  | Default: ``8``

HISTORY\_LENGHT
  | Type: int
  | Default: ``1000``

MAX\_TIME\_WAITING\_BETWEEN\_CONNECTION\_TRY OUTS
  | Type: float
  | Default: ``3.0``

MAX\_TIME\_WAITING\_FOR\_ACKNOWLEDGEMENTS
  | Type: float
  | Default: ``1.0``
  | Maximum time waiting for an acknowledge after sending a message
    which expects one.

SALT\_LENGHT
  | Type: int
  | Default: ``6``

SERVER
  | Type: boolean
  | Default: ``True``
  | Defines if process should start as server or client. If ``False``
    process will start as client.

SERVER\_IP
  | Type: string
  | Default: ``127.0.0.1``
  | If process is client, SERVER\_IP contains the IP to connect to.

SERVER\_UDP\_PORT
  | Type: int
  | Default: ``9999``
  | Contains the UDP port of the server which shall be connected to.

TIME\_BETWEEN\_BURSTS
  | Type: float
  | Default: ``0.01``
  | Time between burst messages (refer to BURST\_NUMBER).

TYPE
  | Type: string
  | Default: ``NETWORK_CONFIG``
  | Specifying the type of configuration. Must be NETWORK\_CONFIG for
    the network config file.

