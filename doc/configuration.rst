Configuration
=============

:ref:`RAFCON` can be configured using two config files, one for
the core and one for the GUI. The config files are automatically
generated (if not existing) on the first run of RAFCON. It is stored in
your home folder: ``~/.config/rafcon/`` with name ``config.yaml`` and
``gui_config.yaml``, respectively. The path can be changed when running
the ``start.py`` script with argument "-c". The syntax used is
`YAML <https://en.wikipedia.org/wiki/YAML>`__.

Core Configuration
------------------

.. _core_config_example:

Example:

A typical config file looks like this:

.. code:: yaml

    TYPE: SM_CONFIG

    LIBRARY_PATHS: {
        "generic": "${RAFCON_LIB_PATH}/generic",
        "tutorials": "${RAFCON_LIB_PATH}/../examples/tutorials",
        "ros": "${RAFCON_LIB_PATH}/../examples/libraries/ros_libraries",
        "turtle_libraries": "${RAFCON_LIB_PATH}/../examples/libraries/turtle_libraries",
        "intermediate_level": "${RAFCON_LIB_PATH}/../examples/functionality_examples"
    }
    LIBRARY_RECOVERY_MODE: False

    LOAD_SM_WITH_CHECKS: True

    STORAGE_PATH_WITH_STATE_NAME: True
    MAX_LENGTH_FOR_STATE_NAME_IN_STORAGE_PATH: None
    NO_PROGRAMMATIC_CHANGE_OF_LIBRARY_STATES_PERFORMED: False

    IN_MEMORY_EXECUTION_HISTORY_ENABLE: True
    FILE_SYSTEM_EXECUTION_HISTORY_ENABLE: True
    EXECUTION_LOG_PATH: "%RAFCON_TEMP_PATH_BASE/execution_logs"
    EXECUTION_LOG_SET_READ_AND_WRITABLE_FOR_ALL: False

    SCRIPT_RECOMPILATION_ON_STATE_EXECUTION: True

.. _core_config_docs:

Documentation:

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
  | A dictionary holding all libraries accessible in RAFCON. The key of
    the dictionary is a unique library identifier. This unique
    identifier will be used as library name, shown as root of the
    library hierarchy in the library tree. The value of the dictionary
    is a relative or absolute path on the file system that is searched
    for libraries. Relative paths are assumed to be relative to the
    config file. Environment variables are also allowed.

LIBRARY\_RECOVERY\_MODE
  | Type: boolean
  | Default: ``False``
  | If this flag is activated, state machine with consistency errors concerning their data ports can be loaded.
    Instead of raising exceptions only errors are printed. Invalid transitions and data-flows will just be removed.
    This mode can be used to fix erroneous state machines.
    Intermediate and expert users can also keep this setting enabled all the time.

LOAD\_SM\_WITH\_CHECKS
  | Type: boolean
  | Default: ``True``
  | If this flag is activated, every state is checked for consistency before loaded.
    If set to false all consistency checks will be skipped. This leads to much faster loading times.
    However, if there are consistency errors RAFCON tries to open the state machines and will fail.

STORAGE\_PATH\_WITH\_STATE\_NAME
  | Type: boolean
  | Default: ``True``
  | If set to True the paths to save states will contain the state names.
    If False only the state IDs will be used to create the storage path.

MAX\_LENGTH\_FOR\_STATE\_NAME\_IN\_STORAGE\_PATH
  | Default: ``None``
  | Unit: number
  | Specifies the maximum length of a state name in the storage path.
    If the state name is longer than the specified value, the state name is truncated.
    If the value is set to None the whole state name is used inside the path.

NO\_PROGRAMMATIC\_CHANGE\_OF\_LIBRARY\_STATES\_PERFORMED
  | Type: boolean
  | Default: ``False``
  | Set this to True if you can make sure that the interface of library states is not programmatically changed anywhere inside your state machines. This will speed up loading of libraries.
    If you use template state machines that insert states during runtime, this must be disabled.

IN\_MEMORY\_EXECUTION\_HISTORY\_ENABLE
  | Type: boolean
  | Default: ``True``
  | Enables execution history. The execution history is required for backward execution and execution logging to the file system.

FILE\_SYSTEM\_EXECUTION\_HISTORY\_ENABLE
  | Type: boolean
  | Default: ``True``
  | Enables the logging of rafcon execution histories to the file system. Every time a statemachine is executed, a python shelve is created in the execution log directory, e.g. ``/tmp/rafcon_execution_logs/rafcon_execution_log_99-Bottles-of-Beer_2017-08-31-16-07-17.shelve``. Some helpful utility functions for working with log files through python are in: ``import rafcon.utils.execution_log``. A tiny tiny code snippet which shows how to use the pandas.DataFrame representation to query the outcomes of a state named ‘CheckFinished’ is here: ``https://rmc-github.robotic.dlr.de/common/rafcon/pull/324#issuecomment-2520``

EXECUTION\_LOG\_PATH:
  | Type: String
  | Default: ``"/tmp/"``
  | Sets the target path of the execution logs

EXECUTION\_LOG\_SET\_READ\_AND\_WRITABLE\_FOR\_ALL:
  | Type: boolean
  | Default: ``False``
  | If True, the file permissions of the log file are set such that all users have read access to this file.

SCRIPT\_RECOMPILATION\_ON\_STATE\_EXECUTION:
  | Type: boolean
  | Default: ``True``
  | If True, the script of an ``ExecutionState`` will be recompiled each time the state is executed, effectively
    resetting all global variables. For reasons of backwards compatibility, the default value is ``True``. It is
    recommended to set the value to ``False``, causing a recompilation only when the execution of a state machine is
    newly started, which is a bit faster and allows to share data between consecutive state executions.
  
GUI Configuration
-----------------

.. _gui_config_example:

A typical config file looks like this:

.. code:: yaml

    TYPE: GUI_CONFIG

    SOURCE_EDITOR_STYLE: rafcon

    GAPHAS_EDITOR_AUTO_FOCUS_OF_ROOT_STATE: True
    ENABLE_CACHING: True
    THEME_DARK_VARIANT: True
    DRAG_N_DROP_WITH_FOCUS: False

    WAYPOINT_SNAP_ANGLE: 45
    WAYPOINT_SNAP_MAX_DIFF_ANGLE: 10
    WAYPOINT_SNAP_MAX_DIFF_PIXEL: 50

    PORT_SNAP_DISTANCE: 5

    LOGGING_SHOW_VERBOSE: False
    LOGGING_SHOW_DEBUG: False
    LOGGING_SHOW_INFO: True
    LOGGING_SHOW_WARNING: True
    LOGGING_SHOW_ERROR: True
    CONSOLE_FOLLOW_LOGGING: True

    LIBRARY_TREE_PATH_HUMAN_READABLE: False
    SUBSTITUTE_STATE_KEEPS_STATE_NAME: True

    MINIMUM_SIZE_FOR_CONTENT: 30
    MAX_VISIBLE_LIBRARY_HIERARCHY: 2
    NO_FULLY_RECURSIVE_LIBRARY_MODEL: True

    USE_ICONS_AS_TAB_LABELS: True

    SHOW_NAMES_ON_DATA_FLOWS: True
    SHOW_CONTENT_LIBRARY_NAME_TRANSPARENCY: 0.5
    ROTATE_NAMES_ON_CONNECTIONS: False

    HISTORY_ENABLED: True

    KEEP_ONLY_STICKY_STATES_OPEN: True

    AUTO_BACKUP_ENABLED: True
    AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL: False
    AUTO_BACKUP_FORCED_STORAGE_INTERVAL: 120
    AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL: 20
    AUTO_RECOVERY_CHECK: False
    AUTO_RECOVERY_LOCK_ENABLED: False

    SESSION_RESTORE_ENABLED: True

    NUMBER_OF_RECENT_OPENED_STATE_MACHINES_STORED: 20

    AUTO_APPLY_SOURCE_CODE_CHANGES: True

    CHECK_PYTHON_FILES_WITH_PYLINT: False

    DEFAULT_EXTERNAL_EDITOR:
    PREFER_EXTERNAL_EDITOR: False

    RESTORE_UNDOCKED_SIDEBARS: True

    FULLSCREEN_SHOW_TOOLBAR: True

    NOTIFICATIONS_MINIMUM_LOG_LEVEL: 30
    NOTIFICATIONS_DURATION: 3

    STATE_SELECTION_INSIDE_LIBRARY_STATE_ENABLED: True
    LIBRARY_TREE_TOOLTIP_INCLUDES_ROOT_STATE_DESCRIPTION: True

    ZOOM_WITH_CTRL: False

    SEMANTIC_DATA_MODE: False
    SHOW_PATH_NAMES_IN_EXECUTION_HISTORY: False
    EXECUTION_TICKER_ENABLED: True
    EXECUTION_TICKER_PATH_DEPTH: 3

    # 300 is equal to glib.PRIORITY_LOW which is is lower than the default gtk priority
    LOGGING_CONSOLE_GTK_PRIORITY: 300

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
        is_start_state:
        - <Control>E
        - <Control><Shift>X
        transition_from_closest_sibling_state: <Control><Shift>C
        transition_to_closest_sibling_state: <Control><Shift>V
        transition_to_parent_state: <Control><Shift>B
        left:
        - <Control>Left
        - <Control><Shift>Left
        new: <Control>N
        open: <Control>O
        open_external_editor: <Control><Shift>Q
        open_library_state_separately: <Control><Shift>space
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

Documentation:

TYPE
  | Type: String-constant
  | Default: ``GUI_CONFIG``
  | Specifying the type of configuration. Must be GUI\_CONFIG for the
    GUI config file.

SOURCE\_EDITOR\_STYLE
  | Type: string
  | Default: ``rafcon``
  | The gtk source view style used in the script editor. Note: You can
    download different styles
    `here <https://wiki.gnome.org/Projects/GtkSourceView/StyleSchemes>`__.
    The scripts have to be downloaded to
    <rafcon package directory>/share/gtksourceview-2.0/styles. "rafcon" is a style
    created to fit to the design of RAFCON.

GAPHAS\_EDITOR\_AUTO\_FOCUS\_OF\_ROOT\_STATE
  | Type: boolean
  | Default: ``True``
  | If RAFCON is started with the Gaphas editor enabled this flag enables an
    initial auto focus of the root state after opening the state machine.
    If you do not like this feature simply disable it (False).

ENABLE\_CACHING:
  | Default: ``True``
  | Enables a accelerating caching feature.

THEME\_DARK\_VARIANT:
  | Default: ``True``
  | If ``True``, a dark theme will be used, else a light theme

PORT\_SNAP\_DISTANCE
  | Default: ``5``
  | Unit: Pixel
  | Maximum distance to a port, at which the moved end of a connection is
    snapped to a port (outcome, input, output, scoped variable).

LOGGING\_SHOW\_VERBOSE
  | Type: boolean
  | Default: ``False``
  | The flag decides to activate the VERBOSE log level in the logging console view.

LOGGING\_SHOW\_DEBUG
  | Type: boolean
  | Default: ``False``
  | The flag decides to activate the DEBUG log level in the logging console view.
    
LOGGING\_SHOW\_INFO
  | Type: boolean
  | Default: ``True``
  | The flag decides to activate the INFO log level in the logging console view.
    
LOGGING\_SHOW\_WARNING
  | Type: boolean
  | Default: ``True``
  | The flag decides to activate the WARNING log level in the logging console view.
    
LOGGING\_SHOW\_ERROR
  | Type: boolean
  | Default: ``True``
  | The flag decides to activate the ERROR log level in the logging console view.

CONSOLE\_FOLLOW\_LOGGING
  | Type: boolean
  | Default: ``True``
  | The flag decides to activate the follow mode in the logging console view and to stay on the last printed logger message.

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
    In the dialogs this can be adapted for each single operation via a check box.
    If the flag is True the name is taken from the original state.
    If the flag is False the name is taken from the state machine that substitutes the original state.

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
  | If True, only icons will be shown in the tabs of the notebooks of the left and right pane. Otherwise the text of the notebook tab is shown as text.

SHOW\_NAMES\_ON\_DATA\_FLOWS
  | Type: boolean
  | Default: ``True``
  | If False, data flow labels will not be shown (helpful if there are
    many data flows)

SHOW\_CONTENT\_LIBRARY\_NAME\_TRANSPARENCY
  | Type: float
  | Default: ``0.5``
  | Set to a value between 0 and 1. Defines the transparency of the name of a LibraryState in the graphical editor,
    of which the content is shown.

ROTATE\_NAMES\_ON\_CONNECTIONS
  | Type: boolean
  | Default: ``False``
  | If True, connection labels will be parallel to the connection.
    Otherwise, they are horizontally aligned.

HISTORY\_ENABLED
  | Type: boolean
  | Default: ``True``
  | If True, an edit history will be created, allowing for undo and redo
    operations.

KEEP\_ONLY\_STICKY\_STATES\_OPEN
  | Type: boolean
  | Default: ``True``
  | If True, only the currently selected state and sticky states are
    open in the "states editor" on the right side. Thus, a newly selected
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
  | If True, the auto backup is performed according to a fixed time
    interval which is defined by
    ``AUTO_BACKUP_FORCED_STORAGE_INTERVAL``. If False, the auto-backup
    is performed dynamically according to
    ``AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL``. This means that RAFCON tries to avoid user disturbances
     by waiting for the case that the user does not perform any changes to the state machine for
    ``AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL`` seconds. If this happens RAFCON will perform a backup.
    Still ``AUTO_BACKUP_FORCED_STORAGE_INTERVAL`` is used as a hard storage interval.
    More information about this can be found on :ref:`Auto Backup`

AUTO\_BACKUP\_FORCED\_STORAGE\_INTERVAL
  | Default: 120
  | Unit: Seconds
  | Time horizon for a forced auto-backup if ``AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL`` is True.

AUTO\_BACKUP\_DYNAMIC\_STORAGE\_INTERVAL
  | Default: 20
  | Unit: Seconds
  | Time horizon after which the auto-backup is triggered if
    there was no modification to the state-machine for an time interval of this size. (only if ``AUTO_BACKUP_ONLY_FIX_FORCED_INTERVAL`` is False)

AUTO\_RECOVERY\_CHECK
  | Default: ``False``
  | If True, the auto back module will check for backups of crashed RAFCON instances. This comfortable feature
    only can be used if the crashed instances or state machines were already
    created with ``AUTO_RECOVERY_LOCK_ENABLED`` and ``AUTO_BACKUP_ENABLED`` set to True.

AUTO\_RECOVERY\_LOCK\_ENABLED:
  | Default: ``False``
  | If True, the auto backup will put lock-files into the respective backup folder
    to label not correctly/cleanly closed state machines and instances.
    The auto recovery check is searching for these lock-files.

SESSION\_RESTORE\_ENABLED:
  | Default: ``True``
  | If True the current session is stored into the runtime configuration and restored
    after restarting RAFCON.

NUMBER\_OF\_RECENT\_OPENED\_STATE\_MACHINES\_STORED:
  | default: 20
  | Maximum number of state machines that can be restored in a session.

AUTO\_APPLY\_SOURCE\_CODE\_CHANGES
  | Default: ``True``
  | If True, RAFCON will apply source code changes on saving a state machine.

CHECK\_PYTHON\_FILES\_WITH\_PYLINT
  | Default: ``False``
  | If True, RAFCON checks the script file with pylint before saving it. In case of an error a message dialog will pop up to warn the user about the error.

DEFAULT\_EXTERNAL\_EDITOR
  | Default: Empty
  | Holds the command for the editor to open the script.py file with, if the user clicks the
    'Open externally' button in the source editor window. The command can be anything
    and results in a shell command with the following pattern: '<DEFAULT\_EXTERNAL\_EDITOR> script.py>'.

PREFER_EXTERNAL_EDITOR
  | Default: ``False``
  | If True, RAFCON will assume that the user always wants to work with a different editor
    than the internal one. If the 'Open externally' button is clicked, the source text is
    locked the whole time and a 'Reload' button reloads the saved file into RAFCON.
    If False, it is recommended to close the externally opened script.py everytime you are
    done editing.

RESTORE\_UNDOCKED\_SIDEBARS
  | Default: ``True``
  | If True, RAFCON will restore undocked windows from the last RAFCON-instance run.

FULLSCREEN\_SHOW\_TOOLBAR
  | Default: ``True``
  | If True, the toolbar with execution and state buttons is shown in fullscreen mode.

NOTIFICATIONS\_MINIMUM\_LOG\_LEVEL
  | Default: ``30``
  | Minimum log level of messages that shell show up in the notification bar. ``40`` corresponds to ``ERROR``,
    ``30`` to ``WARNING``, ``20`` to ``INFO``, ``10`` to ``DEBUG`` and ``5`` to ``VERBOSE``. If this is set to a level
    higher than ``40``, no notifications are shown.

NOTIFICATIONS\_DURATION: 3
  | Default: ``3``
  | Number of seconds a notification is shown. If set to ``0``, the notification must be closed manually.

STATE_SELECTION_INSIDE_LIBRARY_STATE_ENABLED:
  | Default: ``True``
  | If set to True, states inside library states can be selected.

LIBRARY_TREE_TOOLTIP_INCLUDES_ROOT_STATE_DESCRIPTION:
  | Default: ``True``
  | If set to True, tooltip include the root state description text if the hovered library tree element (leaf element) is a real state machine.

ZOOM_WITH_CTRL:
  | Default: ``False``
  | If set to True the user has to press the CTRL button to zoom into a state machine.

SEMANTIC\_DATA\_MODE
  | Default: ``False``
  | If True, RAFCON gives the semantic data editor of each state more vertical space.
    The vertical space is taken from the port/connection widget. This is especially useful, when working a lot with semantic data.

SHOW\_PATH\_NAMES\_IN\_EXECUTION\_HISTORY
  | Default: ``False``
  | If True, RAFCON shows the state paths next to the state names in each execution history entry.

EXECUTION\_TICKER\_ENABLED
  | Default: ``True``
  | If True, the execution ticker will prompt activity into respective widget.

EXECUTION\_TICKER\_PATH\_DEPTH
  | Default: ``3``
  | Number of state names shown in active path (by names) starting from the lowest leaf state as the last
    and cutting away the first and following if to much.

LOGGING\_CONSOLE\_GTK\_PRIORITY:
  | Default: 300
  | Unit: Priority
  | Sets the priority of logging anything to the console widget. The lower the number, the higher the priority. If the priority is too high, than the GUI will lag during execution, as the console widget will than slow down the rendering of gaphas / OpenGL

SHORTCUTS
  | Type: dict
  | Default: see example ``gui_config.yaml`` above
  | Defines the shortcuts of the GUI. The key describes the action
    triggered by the shortcut, the value defines the shortcut(s). There
    can be more than one shortcut registered for one action. See `GTK
    Documentation <https://lazka.github.io/pgi-docs/Gtk-3.0/functions.html#Gtk.accelerator_parse>`__
    about more information about the shortcut parser. Not all
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

Example:

To not destroy the behavior of RAFCON, the default configuration should be used as basis for your extensions. The
following example shows how to add another logging handler, writing all messages to a file:

.. code:: json

    {
        "loggers": {
            "rafcon": {
                "handlers": ["stdout", "stderr", "loggingView", "file"]
            }
        },

        "handlers": {
            "file": {
                "class": "logging.handlers.RotatingFileHandler",
                "formatter": "default",
                "filename": "/tmp/rafcon.log",
                "maxBytes": 1024,
                "backupCount": 3
            }
        },
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

Example:

The default ``network_config.file`` looks like:

.. code:: yaml

    TYPE: NETWORK_CONFIG
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
    BURST_NUMBER: 1
    CLIENT_UDP_PORT: 7777

.. _monitoring_plugin_docs:

Documentation:

TYPE
  | Type: string
  | Default: ``NETWORK_CONFIG``
  | Specifying the type of configuration. Must be NETWORK\_CONFIG for
    the network config file.

ENABLED
  | Type: boolean
  | Default: ``True``
  | The monitoring plugin is only used if this value is set to True.

HASH\_LENGTH
  | Type: int
  | Default: ``8``
  | If you have many different message contents, increase this number.

HISTORY\_LENGTH
  | Type: int
  | Default: ``1000``

MAX\_TIME\_WAITING\_BETWEEN\_CONNECTION\_TRY\_OUTS
  | Type: float
  | Default: ``3.0``

MAX\_TIME\_WAITING\_FOR\_ACKNOWLEDGEMENTS
  | Type: float
  | Default: ``1.0``
  | Maximum waiting time for an acknowledgement after sending a message
    which expects one.

SALT\_LENGHT
  | Type: int
  | Default: ``6``

SERVER
  | Type: boolean
  | Default: ``True``
  | Defines if the RAFCON instance should start as server or client. If ``False``
    process will start as client.

SERVER\_IP
  | Type: string
  | Default: ``127.0.0.1``
  | If RAFCON is started as client, SERVER\_IP contains the IP to connect to.

SERVER\_UDP\_PORT
  | Type: int
  | Default: ``9999``
  | Contains the UDP port of the server which shall be connected to.

TIME\_BETWEEN\_BURSTS
  | Type: float
  | Default: ``0.01``
  | Time between burst messages (refer to BURST\_NUMBER).

BURST\_NUMBER
  | Type: int
  | Default: ``1``
  | Amount of messages with the same content which shall be send to
    ensure the communication.

CLIENT\_UDP\_PORT
  | Type: int
  | Default: ``7777``
  | Contains the UDP port of the client

