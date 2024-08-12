Changelog
=========

Information about :ref:`RAFCON` changes in each release will be published here. More
details can be found in the `GIT commit log <https://github.com/DLR-RM/RAFCON/commits/master>`__.

2.1.4
""""""
    - Bug fixes:
        - Fixed bug where data port would not reset properly
        - Fixed bug when relocating an existing data flow

    - Miscellaneous:
        - Added proper deletion of gvm variables on shutdown

2.1.3
""""""
    - Bug fixes:
        - Fixed AboutDialog logo image path
        - Bringing back state editor guide lines
        - Fixed issue that children states can be dragged out of parent states (same for state titles)
        - Fixed issue where CTRL+Z in Source Editor or Description would undo twice instead of once
        - Fixed cramped Linkage Overview widget (now properly displays the logical linkage)
        - Fixes bug when trying to connect a data port outside of the parent state

    - Miscellaneous:
        - Updated docs text, configuration and layout for webpage

2.1.2
""""""
    - Bug fixes:
        - Fixed bug for logic connectors
        - Added logo to about dialog

    - Miscellaneous:
        - Updated pdm.lock
        - Added minimal rafcon core tutorial in docs
        - Updated formatting and deprecation for docs
        - Updated citation formatting

2.1.1
""""""
    - Bug fixes:
        - Updated deprecated dependency to jsonconversion
        - Added new test state machine for backward compatibility testing

2.1.0
""""""
    - Features:
        - Added option to disable popups in the config file
        - Display warning when saving new state machine (and overwriting) in already existing folder

    - Bug fixes:
        - Fixing segmentation fault when changing state type in gui via state editor
        - Fixing decider node is not preempted in concurrency state
        - Fixed warnings and bugs in unit tests
        - More minor bugfixes
        
    - Miscellaneous:
        - Checked dependencies and deprecations for libraries
        - Updated shebang versions to python3
        - Removed some warnings resulting from old python conventions

2.0.0
""""""
    - Features:
        - Switch from setup.py to pyproject.toml and pdm for the python package management
        - Add bump2version to avoid human errors when updating the rafcon version
        - Added auto-layout functionality (as a first version)

1.3.0
"""""""
    - Features:
        - Add possibility to only release rafcon-core

1.2.1
"""""""
    - Features:
        - Add __main__.py

1.2.0
"""""""
    - Features:
        - Support python 3.10 

1.1.1
"""""""
    - Bug Fixes:
    - Fix a few GUI bugs

1.1.0
"""""""
    - Features:
        - Add skip & skip all buttons in the dialog of the broken libraries during loading a broken session
        - Set the default directory of the dialog during saving a state machine in accordance with the chosen library in the library tree
        - Create the data flows & data ports automatically in the nested states
        - Create the data port automatically when the other state does not have one during connecting the data flows
        - Support waypoints for data flows
        - Custom background color for states

    - Bug Fixes:
        - Fix many minor GUI bugs


1.0.1
"""""""
    - Bug Fixes:
        - Fix the default primary font name


1.0.0
"""""""
    - Features:
        - Not supporting Python 2 anymore
        - Run this state
        - Only run this state
        - Add singleton pattern
        - Add new hooks before and after running each state
        - Add new memory profiling test to assert the memory leak during running sequential & concurrency state machines
        - Update gaphas to 2.1.2
        - Update libsass to the latest version of dart sass
        - Replace gtkmvc3 with two separated mvc and observer patterns
        - Run this state

    - Bug Fixes:
        - Fix GUI freezing during keeping undo/redo shortcuts

    - Miscellaneous:
        - Remove last update field to improve versioning control
        - Remove a big amount of the dead codes and comments


0.15.4
"""""""

- Bug Fixes:
    - Support custom design


0.15.3
"""""""

- Bug Fixes:
    - Fix bug in LoggingView, which freezes RAFCON


0.15.2
"""""""

- Bug Fixes:
    - Make operations on the logging console thread-safe
    - Define a new GUI config called 'MAX_LOGGING_BUFFER_LINES' that determines the maximum lines of the logging buffer. If the number of lines exceeds the config value, the old value will be deleted automatically via clipping.


0.15.1
"""""""

- Bug Fixes:
    - Call 'show_notification' via 'idle_add'


0.15.0
"""""""

- Features:
    - Libraries can now be renamed and relocated. This includes single libraries, library folders and library root keys
    - Ctrl+F can be used to search for states
    - Missing libraries are supported better. In case a library cannot be found, the transitions and data-flows are preserved and added to the dummy-state, which is inserted instead of the library. Furthermore, the dummy-state has the same position and size as the old library state.
    - New execution-history structure: Define specific consumers for in-memory-execution-history and file-system execution history. Furthermore, another hook was defined such that RAFCON plugins can be used to define further consumers. Watch out: the config values for controlling the execution history changed


0.14.11
"""""""

- Features:
    - Add search bar for lookup through state machine libraries
    - Add find usage for finding the usages of state machine libraries

- Bug Fixes:
    - Fix handling of library interface change


0.14.10
"""""""

- Features:
    - Add new config (``RAISE_ERROR_ON_MISSING_LIBRARY_STATES``) to make Rafcon raise error when loading


0.14.9
""""""

- Features:
    - add states for execution control


0.14.8
""""""

- Bug Fixes:
    - Fix py2 support


0.14.7
""""""

- Features:
    - increase test coverage
    - add gitlab runners support
    - differentiate between py3 and py2 dependencies in setup.py
    - differentiate between EXECUTION_LOG_ENABLE and EXECUTION_LOG_TO_FILESYSTEM_ENABLE config options i.e. keep memory footprint of RAFCON constant
    - add memory leak test
    - Fix race condition in 'call_gui_callback'


0.14.6
""""""

- Miscellaneous:
    - fix buggy pypi upload


0.14.5
""""""

- Bug Fixes:
    - execution log viewer now works via released script in bin folder


0.14.4
""""""

- Features:
    - :issue_gh:`290` paste state at current mouse position (both via context menu and shortcut) @CSuerig
    - add state at context menu position when using context menu to add states @CSuerig


0.14.3
""""""

Maintenance release.


0.14.2
""""""

- Features:

  - Replace ``SCRIPT_COMPILE_ON_FILESYSTEM_LOAD`` in favor of ``SCRIPT_RECOMPILATION_ON_STATE_EXECUTION``. See the documentation of the configuration for details.


- Bug Fixes:

  - :issue_gh:`28` Setting of external editor via dialog does not work
  - :issue_ghe:`790` gui_config.yaml not saved anymore automatically
  - Make tests run with pytest-mock>=1.11.2
  - Add compatibility with pylint>=2.4
  - Positions of panes should be restored correctly
  - Fix several deprecation warnings


- Miscellaneous:

  - do not test Python 3.4 on Jenkins
  - Coverage test only on ``develop`` and ``master`` branch
  - prepare for new ``yaml_configuration`` release


0.14.1
""""""

- Bug Fixes:

  - :issue_ghe:`774` python setup.py build_sass not working
  - :issue_gh:`26` python3's "__pycache__" folder chrashes loading of examples


0.14.0
""""""

- Features:

  - new notification bar, informing about important log entries (configurable), fixes :issue_ghe:`288`
  - Fullscreen mode: optionally show toolbar (``FULLSCREEN_SHOW_TOOLBAR`` option), show notifications


- Improvements:

  - most ``[PyGTK]DeprecatedWarning``\s are fixed
  - graphical editor: minor performance optimizations
  - specify separators for JSON files: Python 3.4 no longer changes the whitespaces in state machine files
  - override builtins string in JSON files: state machine files generated by Python 2 and 3 are now fully identical
  - code coverage report in Jenkins
  - shows RAFCON log messages during installation
  - parallel test runs on Jenkins
  - :issue_gh:`21` Do not store semantic data if not available
  - :issue_ghe:`665` Keep root state position when collapsing left sidebar
  - better defaults:

    - root state is named "root state", further states "[state type] [states counter]"
    - script of ``ExecutionState``\s uses more RAFCON features (``preemptive_wait``, return outcome name)
    - name of states uses full width of state

  - provide RAFCON wheel file
  - make installation more robust, especially against missing font files
  - simplify installation process
  - clear separation in handling of ``data_files`` and ``package_files``
  - create translation files automatically when building dist packages
  - refactored many parts of modification history


- Bug Fixes:

  - :issue_gh:`20` program icon in task bar missing since version 0.13.x
  - :issue_ghe:`665` state type dropdown menu prevents state editor widget to shrink
  - :issue_ghe:`694` json library in python 3.6 writes one-line json files
  - :issue_ghe:`721` Correct execution history logging
  - :issue_ghe:`726` State with self-transition cannot be substituted
  - :issue_ghe:`727` Sticky-Flag in States-Editor can cause crash if state type change is performed
  - :issue_ghe:`755` Positions of outcomes are not always updated
  - fixes bug of "locked" global variable during multithreading access
  - use a safe loader for GUI config file
  - fix handling of symlinks in LibraryManager
  - better support of virtual envs


- Changes:

  - drop support for BuildBot
  - Jenkinsfile: tests are now also run under Python 3.6


- Miscellaneous:

  - new ``gui`` ficture for simplifying GUI tests
  - refactor GUI tests using the ``gui`` fixture
  - documentation on how to write tests and how to use ``gui`` fixture


Patch releases 0.13.\*
----------------------

0.13.8
""""""

- Improvements:

  - use with statement instead af acquire/release
  - dedicated 'unstable' marks for python 2.7 and 3.x; these marks can be used to filter out tests
  - use Python warning module with custom ``RAFCONDeprecationWarning`` for deprecated usages
  - the documentation can again be build on Read The Docs (at least the build of the API docs was corrupt since v0.13)
  - tooltip of library tree include root state description text of libraries
  - Jenkins integration
  - test adaptions so that they can be parallelized
  - added `seqm.yaml` for tracking software engineering quality management (SEQM) requirements (DLR internal)


- Bug Fixes:

  - :issue_gh:`12` Error when switching from python2 to python3
  - :issue_gh:`18` State machines with library states cannot be opened if show flag is set to True
  - :issue_ghe:`683` rafcon can now be closed properly via signal
  - :issue_ghe:`712` Paste of Port into selected state is not possible
  - :issue_ghe:`711` Gaphas does not allow data flows from one state to itself
  - :issue_ghe:`717` States that have data-flows from its output to its input crash gahpas while state type change
  - fix broken links in documentation
  - use correct version and year in documentation


- Changes:
  - pyyaml is not a dependency anymore, as it is now a dependency of yaml_configuration


0.13.7
""""""

- Improvements:

  - add tox integration

    - run tests under Python interpreters 2.7, 3.4, 3.5, 3.6, 3.7
    - run tests with coverage
    - build documentation and check links
    - check sdist

  - optimize setup_requires in setup.py (faster installation)
  - mark unreliable tests as unstable
  - define timeouts for all tests

- Bug Fixes:

  - :issue_ghe:`689` rafcon cannot run without numpy
  - :issue_ghe:`679` error message when connecting data flow
  - fix severe threading bug in call_gui_callback, which could lead to a complete freeze of a state machine


0.13.6
""""""

- Features:

  - add ExecutionTicker to see activity of state machine with high hierarchy depth

- Improvements:

  - changing states (adding or removing) during step mode works now

- Bug Fixes:

  - :issue_ghe:`678` script validation does not work
  - :issue_ghe:`663` cannot rename connected data port of type object
  - :issue_ghe:`684` ``test_simple_execution_model_and_core_destruct_with_gui`` fails when running core & gui tests in a row
  - fix pause and step mode behavior
  - installation of fonts under Python 3
  - various test fixed for Python 3


0.13.5
""""""

- Bug Fixes:

  - Continue installation of none-existing fonts in case that one font was already installed


0.13.4
""""""

- Bug Fixes:

  - Fix installation of not-existing fonts
  - :issue_ghe:`660` tab of executed state machine stays green
  - :issue_ghe:`667` dialog "saving state as library" not working properly
  - :issue_ghe:`664` cleaning of execution history does not work
  - :issue_ghe:`668` adding a state as template screws up meta data
  - Fix rescaling factor**2 if adding libraries as template
  - :issue_ghe:`631` Cut of multiple states creates various problems

- Changes:

  - Increase any MAX_VISIBLE_LIBRARY_HIERARCHY value to be minimal 2 -> for performance the aim is to allow lower values again


0.13.3
""""""

- Changes:

  - Release correct style files


0.13.2
""""""

- Features:

  - The right click menu of library state can be used to select and focus respective library tree element

- Bug Fixes:

  - :issue_ghe:`658` crash in load_state_machine
  - run correct command for updating font cache

- Changes:

  - Replaced font "DIN Next LT Pro" by "Source Sans Pro"


0.13.1
""""""

- Bug Fixes: Fix installation


0.13.0
""""""

This is a shiny new minor release of RAFCON. Finally, Python 3 (>=3.4) is supported, while Python 2.7 can still be
used, thanks to the ``future`` packet. With this, we also ported the GUI from GTK+ 2 to GTK+ 3, allowing for better
styling. Of course, there are many more improvements and bug fixes:

- Features:

  - RAFCON is now compatible to Python 3
  - GTK+ 2 to GTK+ 3 port of the RAFCON GUI
  - Better styling including a HeaderBar
  - Alternative light theme! (GUI config option ``THEME_DARK_VARIANT``)

- Improvements:

  - :issue_ghe:`117` Make GUI resizeable on all edges and corners
  - :issue_ghe:`610` Provide CITATION.cff to make software citable
  - :issue_ghe:`619` Provide and install \*.desktop file
  - :issue_ghe:`621` Provide full license text
  - :issue_ghe:`636` No exception when closing RAFCON and a state machine is still running
  - :issue_ghe:`637` No exception when closing a state machine tab, when it still runs
  - :issue_ghe:`640` Backward compatibility test runs with various python versions now
  - :issue_ghe:`646` Library roots can be added and removed inside the library tree
  - The installation should now work from a blank virtualenv
  - The documentation about the release steps has been extended

- Bug Fixes:

  - :issue_ghe:`596` External editor does not remember the handed command and also does not lock the embedded editor
  - :issue_ghe:`617` Invalid DataFlow by DataFlowWidget
  - :issue_ghe:`618` semantic data strings get scrambled/obfuscated in execution history log
    fixed by pull request :issue_ghe:`626` fix(execution_log): unpickle semantic data
  - :issue_ghe:`624` Debug console: cursor is not positioned at the point were it is clicked on
  - :issue_ghe:`627` Generic library state machines need Gtk2 to gtk3 conversion
  - :issue_ghe:`638` Exiting Fullscreen mode hides the graphical editor
  - :issue_ghe:`644` "Substitute state as template" creates problems if not all models are recursive created

- Changes:

  - Redundant libraries are marked as deprecated
  - No more "+"-icon next to state machine tabs to add a new state machine (related to :issue_ghe:`639`)
  - Remove old OpenGL GraphicalEditor
  - Remove deprecated entry points ``rafcon_start`` and ``rafcon_start_gui``


Patch releases 0.12.\*
----------------------

0.12.25
"""""""

- Improvements:

  - A ``DataPort`` with data type ``object`` can now be connected to any other ``DataPort`` (:issue_ghe:`422`, :issue_ghe:`525`)
  - :issue_ghe:`602` Hide menu entries without function
  - Handle exceptions of the OpenGL graphical editor gracefully => do not depend on ``gtkglext``

- Bug Fixes:

  - no more ``GtkWarning`` in stdout
  - `GitHub Issue #4 <https://github.com/DLR-RM/RAFCON/issues/4>`__ GTK theme does not exist


0.12.24
"""""""

- Improvements:

    - Update documentation regarding installation

- Bug Fixes:

    - Installation of mo-files (for language support) works


0.12.23
"""""""

- Improvements:

  - Update documentation regarding installation
  - Update rafcon dependencies in setup.py

- Bug Fixes:

  - API: ``AttributeError`` when passing ``DeciderState`` to constructor of ``BarrierConcurrencyState``
  - Installation of mo-files (for language support) works


0.12.22
"""""""

- Features:

  - :issue_ghe:`581` Utility shortcuts to add transitions from selected state to parent default outcome and sibling states

- Improvements:

  - redraw graphical editor if connections are removed
  - extend German RAFCON translation
  - extend Developer's Guide by how-to on translating RAFCON
  - API: ``add_state`` is adapting the passed ``state.state_id`` automatically in case of conflicts
    instead of raising an ``AttributeError``

- Bug Fixes:

  - :issue_ghe:`455` Proportional resizing states now works properly
  - :issue_ghe:`538` Many error outputs when changing MAX_VISIBLE_LIBRARY_HIERARCHY
  - :issue_ghe:`541` Where are the magnet lines gone?
  - :issue_ghe:`551` Prevent RAFCON from restarting if installation of fonts fails
  - :issue_ghe:`571` Wrong rendering of scoped variables
  - :issue_ghe:`580` update font installation
  - :issue_ghe:`584` Opening a external source editor fails for a never set active state machine id
  - :issue_ghe:`586` Ungroup of a state with data flows in between of it child states twice in the same hierarchy
    creates corrupt state machine or fails
  - stepping works inside library and concurrency states
  - :issue_ghe:`589` decider state can be deleted
  - make i18n work


0.12.21
"""""""

- Features:
  - new save state machine as menu item for root state right click menu to offer direct 'save as library' operations

- Improvements:

  - :issue_ghe:`579` Integrate external execution log viewer


- Bug Fixes:

  - :issue_ghe:`574` Group fails if it includes data flows between the grouped states or scoped variables

0.12.20
"""""""

- Features:

  - maintenance release

0.12.19
"""""""

- Bug Fixes:

  - fix setup.py, sdist now working on pypi

0.12.18
"""""""

- Features:

  - new shortcut open library state separately as state machine by default on 'Shift+Ctrl+Space' (shortcut works for multiple states, too)

- Improvements:

  - Provides proper PyCharm config files (in the `.idea` folder)
  - update menu item labels
  - updated rst documentation

- Bug Fixes:

  - recent opened state machine list no more miss paths
  - :issue_ghe:`550` Gaphas cairo.Error: invalid value (typically too big) for the size of the input (surface, pattern, etc.)
  - :issue_ghe:`564` Zoom onto mouse position
  - handle config option `ZOOM_WITH_CTRL` properly

0.12.17
"""""""

- Improvements:

  - example state machines and generic libraries get now installed via pypi


0.12.16
"""""""

- Improvements:

  - default config file extended

0.12.15
"""""""

- Improvements:

  - PYTHONUSERBASE added to search path list for gtk style files

0.12.14
"""""""

- Improvements:

  - library_manager: increase performance of loading libraries by caching a list of all loaded libraries
  - gaphas editor: use new meta data hash method to speed up loading time

0.12.13
"""""""

- Improvements:

  - the column headers of state machine tree now can be used to sort the items according state name, ID or type
  - more user friendly interface for tree and list view widgets e.g. data ports, outcomes and semantic data
    -> scrollbar adjustment and selections are moving much less and try to stay in the front of respective widget
  - correct tab motion to be more accurate
  - execution_history widget shows more visible chars per data port


0.12.12
"""""""

- Improvements:

  - :issue_ghe:`530` automatically focus and adapt position of root state for fresh initiated state machines
    issue title was "Root state out of focus and badly positioned"
  - :issue_ghe:`543` Changing default option for library name while saving
    -> for the default folder name white space are replaced with underscores and all is lower case
  - also default library state name is now the folder name with replaced underscores with white spaces


- Bug Fixes:

  - :issue_ghe:`527` RAFCON GUI loops while startup if HOME environment variable is not defined
    -> a error message pointing on respective missing environment variable is added
  - :issue_ghe:`539` grouping of states outcome transitions are not fully recovers (now bug is covered by test)
  - :issue_ghe:`515` source editor does not show end of lines (finally)


0.12.11
"""""""

- Improvements:

  - :issue_ghe:`529` accelerate the follow mode switch for many logger messages
  - dynamic insertion of states during state execution is working and tested
  - secure dynamic modification of state machines while runtime by test created in
    pull request :issue_ghe:`535` Dynamic insertion of states during execution

- Bug Fixes:

  - :issue_ghe:`515` source editor does not show end of lines (partly)
  - :issue_ghe:`533` States inside library states cannot be selected
  - :issue_ghe:`528` execution history destruction does not lead to max recursion depth


0.12.10
"""""""

- Features:

  - :issue_ghe:`520` Debug Console keeps track of last logger message if the follow mode is enabled

- Improvements:

  - in pull request :issue_ghe:`523` refactoring of debug console  for more intuitive and robust behavior
    e.g. persistent cursor position
  - :issue_ghe:`516` source editor does not show line of cursor after apply if the script is big

- Bug Fixes:

  - :issue_ghe:`519` rafcon freezes while opening a state machine
    - solved in pull request :issue_ghe:`524` history elements hold direct state reference
  - :issue_ghe:`514` text in entry widget of port not visible during editing (arrow key press left-right helps)
    - the issue was not fully resolved but improved

0.12.9
""""""

- Improvements:

  - container state API can adjust output_data by new method write_output_data
  - more robust execution history tree
  - performance improvement by deleting gaphas views at once for recursive state destruction's

- Bug Fixes:

  - :issue_ghe:`521` Strange gaphas logs during deletion of a state
  - fix gaphas exceptions if state machine selection holds elements which gaphas has not drawn

0.12.8
""""""

- Feature:

  - start RAFCON with `rafcon` instead of `rafcon_start_gui` or `rafcon_core` instead of `rafcon_start` (old
    commands are still working)

- Improvements:

  - buttons to forcefully lock or unlock a global variable
  - global variable manager logger messages got new failure warning messages
  - copy/paste for semantic data elements
  - new config value SHOW_PATH_NAMES_IN_EXECUTION_HISTORY
  - make library path in state editor overview selectable

- Bug Fixes:

  - :issue_ghe:`503` scoped variable looks weird
  - :issue_ghe:`505` clean up profiler flag in config
  - :issue_ghe:`506` root state input ports leave ugly stripes behind
  - :issue_ghe:`501` transition is not selectable if it is drawn over state
  - :issue_ghe:`512` execution of second state machine cause freeze of stop on previous state machine was not successful
  - :issue_ghe:`514` text in entry widget of port not visible during editing
  - fix state machine tree remove library state
  - no deadlocks when locking a global variable two times
  - :issue_ghe:`502` changing data ports not possible
  - fix state element weakref parent assigenment in case of tolerating a invalid data flow


0.12.7
""""""

- Improvements:

  - updated documentation
  - use verbose logging level instead of prints for modification history debug prints


0.12.6
""""""

- Feature:

  - tests folder is now released as well

- Bug Fixes:

  - fix open-gl support for show-content to support fast state machine exploration (also into all leaf-states by zoom)
  - library state can be removed also when those are showing content



0.12.5
""""""

- Feature

  - new log level "VERBOSE", intended for development purposes
  - state machines can now be baked (a snapshot of the state machine with all libraries can be saved)
  - Graphviz can now be used to debug gtkmvc notifications and signals

- Improvements:

  - Gtk priority of logging output to the console view is now customizable via the gui_config
  - better plugin support of changes to the state-editor tabs
  - gaphas combines now complex meta data actions in one meta data changed signal -> one undo/redo-Action

- Bug Fixes:

  - :issue_ghe:`484` label handles are hard to grasp
  - :issue_ghe:`486` Gaphas is not emitting meta data signal if NameView is moved
  - quick fix for not working "state type change" in combination with library states (which was based on respective
    object destruction while those operations) -> will be fully solved in :issue_ghe:`493`
  - quick fix for not set or too late set of active state machine id -> will be fully solved in :issue_ghe:`495`
  - fix meta data for undo/redo of add object operations
  - fix exception handling, causing issues with the graphical editor when invalid connection were created
  - When hovering the menu bar, an exception was printed


0.12.4
""""""

- Improvements:

  - Provide a `PULL_REQUEST_TEMPLATE` for pull requests opened in GitHub
  - Optimize updates/redrawing of graphical editor

- Bug Fixes:

  - :issue_ghe:`414` state machines with libraries cannot be closed


0.12.3
""""""

- Feature

  - The env variable :envvar:`RAFCON_START_MINIMIZED` allows to start RAFCON minimized, which is helpful when running
    the tests

- Improvements:

  - :issue_ghe:`414` Memory optimizations: The memory usage should no longer increase over time, as unused objects are now freed
  - A new/extended test verifies the correct destruction of removed elements
  - Optimize NameView font size calculations, noticeable during zooming
  - ports outside of the visible view are no longer drawn, which increases the performance, especially while
    zooming in large state machines
  - Hash calculations of state machines
  - Placement of NameView
  - drawing of connections, ports and labels, especially when deeply nested
  - :issue_ghe:`469` unit test refactorings

- Bug Fixes:

  - :issue_ghe:`459` execution_log utils; backward compatibility missing and :issue_ghe:`458` ReturnItem
  - :issue_ghe:`454` group/ungroup is not preserving meta data recursively
  - :issue_ghe:`452` Session restore, gaphas and extended controller causes exception when closing RAFCON
  - :issue_ghe:`450` Names of states inside a library become smaller
  - :issue_ghe:`447` Hashes of state machine in storage different then the reopened state machine after saving it
  - :issue_ghe:`449` ports (of transitions or data flows) cannot be moved
  - :issue_ghe:`471` selection of states in hierarchies >= 5 not possible


0.12.2
""""""

- New Features:

  - Fix logging for library state execution

- Improvements:

  - Improve execution logging (semantic data is supported now)
  - :issue_ghe:`445` Tests need to ensure correct import order for GUI singletons

- Bug Fixes:

  - :issue_ghe:`446` "show content" leads to sm marked as modified


0.12.1
""""""

- New Features:

  - Semantic data editor supports external editor
  - Transparency of library states improved when content is shown

- Improvements:

  - :issue_ghe:`415` Increase visibility of library content

- Bug Fixes:

  - :issue_ghe:`378` Editing default values does not work sometimes


0.12.0
""""""

- New Features:

  - Semantic meta data editor and storage for every state
  - :issue_ghe:`411` Allow outputting data from preempted states

- Bug Fixes:

  - :issue_ghe:`426` Again meta data of library ports are screwed after insertion
  - :issue_ghe:`425` Connection via points not visible
  - :issue_ghe:`424` Wrong path for tooltip for state machines editor tabs
  - :issue_ghe:`431` Test for recently opened state machine fails
  - :issue_ghe:`430` Selection test fails



Patch releases 0.11.\*
----------------------

0.11.6
""""""

- Bug Fixes:

  - :issue_ghe:`428` fix recursion problem in execution log viewer
  - :issue_ghe:`427` Middle click on state machine tab label close wrong state machine
  - :issue_ghe:`419` wrong outcome data in execution history

- Improvements:

  - :issue_ghe:`411` Allow outputting data from preempted states
  - drag'n drop with focus can be enabled and disabled by using the gui config flag DRAG_N_DROP_WITH_FOCUS
  - graphical editor add way points around the state for self transitions as support for the user
  - refactor state machines editor tab click methods and small fixing
  - better on double click focus by gaphas editor and now also triggered by state machine tree

0.11.5
""""""

- Bug Fixes:
  - :issue_ghe:`421` RAFCON does not remember window size after closing -> final part

0.11.4
""""""

- New Features:

  - Move into viewport: Double click on elements in several widgets cause the element to moved into the viewport
    (not yet supported by all widgets)
  - Usage of selection modifiers (e.g. <Ctrl>, <Shift>) should now be more consistent
  - Ports in the graphical editor can now be selection
  - The port selection is synchronized between the graphical editor and the other widgets
  - Ports can be removed from within the graphical editor

- Improvements:

  - Refactoring of the selection
  - Unit tests for selection
  - :issue_ghe:`411` Allow outputting data from preempted states
  - :issue_ghe:`410` Refactor selection
  - :issue_ghe:`403` Incomes and outcomes cannot be differentiated visually

- Bug Fixes:

  - Memory leak fixes
  - :issue_ghe:`402` Connections end in nowhere
  - :issue_ghe:`417` ports of root state do not move with roots state
  - :issue_ghe:`421` RAFCON does not remeber window size after closing -> first part

0.11.3
""""""

- Improvements:

  - :issue_ghe:`405` Possibility to zoom in and out while drawing a connection
  - :issue_ghe:`404` Possibility to scroll left and right in graphical editor
  - :issue_ghe:`403` Incomes and outcomes cannot be differentiated visually

- Bug Fixes:

  - :issue_ghe:`412` global variables cannot be removed
  - :issue_ghe:`413` tree view controller error

0.11.2
""""""

- Improvements:

  - meta data scaling more robust and protect other elements from side effects of it

- Bug Fixes:

  - :issue_ghe:`393` $HOME/.config/rafcon is not generated initially + tests
  - :issue_ghe:`406` Empty library root state without child states cause meta data resize problems with side effects in
    gaphas drawing

0.11.1
""""""

- New Features:

  - :issue_ghe:`384` add "Collapse all" button for library manager and enable the feature for the state machine tree, too

- Improvements:

  - port position default values

- Bug Fixes:

  - Fix issues when copying/converting logical or data ports with clipboard while cut/copy/paste
  - Fix library state port position scaling after adding
  - Fix gaphas viewer problems with undo/redo of complex actions like copy and paste or add/remove of ports
  - :issue_ghe:`10` Fully integrate modification history with gaphas

0.11.0
""""""

- New Features:

  - "Session restore" by default enabled
  - :issue_ghe:`364` "Open Recent" recently opened state state machines sub menu in menu bar under sub-menu Files
  - "Save as copy" in menu bar under sub-menu Files
  - "Show library content" supported for gaphas graphical viewer
  - The inner library states can be selected, copied and used to run the execution from or to this state,
    see :issue_ghe:`366` and :issue_ghe:`367`, too
  - :issue_ghe:`255` The state machine tree shows inner library states, too, and can be used to explore all "leaf"-states
  - Storage format can be adapted by the user (e.g. names of states in paths and there length)
  - The library manager widget/tree supports modifications by right click (remove library, add/remove library roots)
  - Execution tool-bar supports buttons for run to- and run from-state (like right click menu, too)

- Improvements:

  - Refactoring of "Save state as state machine/library"
  - Better default position meta data for states in graphical viewer
  - Proper resize of graphical meta data for complex actions and show library content
  - :issue_ghe:`369` Storage/Load module for state machines more flexible and robust
  - Storage module supports the user to store state machines without platform specific file system format conflicts
  - :issue_ghe:`365` substitute widget in now scrollable
  - The gtkmvc version 1.99.2 is fully supported (:issue_ghe:`388` corrected version in older releases)

- Bug Fixes:

  :issue_ghe:`382` Currently active state machine not correct
  :issue_ghe:`362` Data flows between scoped variables
  :issue_ghe:`354` Meta data broken when adding state as template to state machine
  :issue_ghe:`353` Label not shown when adding state from library

Patch releases 0.10.\*
----------------------

0.10.3
""""""

- Bug Fixes:

  - File Chooser crashed if the same folder was added to the shortcut_folders twice

0.10.2
""""""

- Bug Fixes:

  - :issue_ghe:`385` If runtime config is newly created the last open path is empty and now state machine could be saved

0.10.1
""""""

- Bug Fixes:

  - make execution logs compatible with execution log viewer again


0.10.0
""""""

- Improvements:

  - complex actions(copy & paste, resize) are properly handled in gaphas and in the modification history
  - :issue_ghe:`342` drag and drop now drops the state at the mouse position

- Bug Fixes:

  - show library content for OpenGL works again
  - add as template works again
  - :issue_ghe:`343` Text field does not follow cursor

Patch releases 0.9.\*
---------------------

0.9.8
"""""

- Improvements:

  - execution history can be logged and is configurable via the config.yaml

0.9.7
"""""

- Improvements

  - logging is configured with a JSON file
  - logging configuration can be specified by a user and the env variable :envvar:`RAFCON_LOGGING_CONF`
  - :issue_ghe:`336`: Use custom popup menu in state machine editor to quickly navigate in open state machines

- Bug Fixes

  - :issue_ghe:`349` Save as library functionality erroneous
  - :issue_ghe:`314` Recursion limit reached when including top statemachine as replacement for missing state machine
  - :issue_ghe:`341` Reload only selected state machine
  - :issue_ghe:`339` Only save the statemachine.json
  - :issue_ghe:`338` Selecting a library state should show the data ports widget per default
  - :issue_ghe:`327` State machines are not properly selected
  - :issue_ghe:`337` Pressing the right arrow in the state machine editor opens a new state machine
  - :issue_ghe:`346` Barrier State cannot be deleted

0.9.6
"""""

- Bug fixes

  - fix step mode

0.9.5
"""""

- Bug fixes

  - runtime value flag of library states can be set again
  - add missing files of last release

0.9.4
"""""

- Bug Fixes

  - change VERSION file install rule to: ./VERSION => ./VERSION

0.9.3
"""""

- Bug Fixes

  - Fix missing VERSION file

0.9.2
"""""

- Improvements

  - Add rmpm env test
  - First version of setup.py
  - Version determination now in rafcon.__init__.py
  - Add another plugin hook, which is called each time a state machine finishes its execution

- Bug Fixes

  - Fix complex issues including the decider state
  - :issue_ghe:`322` Group/Ungroup is not working when performed on childs of a BarrierConcurrencyState
  - :issue_ghe:`326` RAFCON_INSTANCE_LOCK_FILE exception

0.9.1
"""""

- Bug Fix
  - fix bad storage format in combination with wrong jsonconversion version

0.9.0
"""""

- Improvements

  - Consistent storage format
  - Renamed modules: mvc to gui and core to statemachine
  - External editor
  - Substitute State
  - Open externally
  - Save selected state as library
  - Meta data convert methods with clear interface from Gaphas to OpenGL and OpenGL to Gaphas -> only one type of meta data hold
  - Undocked side bars can be restored automatically after restart if `RESTORE_UNDOCKED_SIDEBARS` is set to True.

- Bug Fixes

  - :issue_ghe:`299`: State labels can be placed outside the state borders
  - :issue_ghe:`298`: Child states can be placed outside hierarchy states
  - :issue_ghe:`45`: Size of GUI cannot be changed
  - :issue_ghe:`284`: Core does not check the type of the default values
  - :issue_ghe:`282`: Input and output data port default_value check does not cover all cases
  - :issue_ghe:`280`: List of tuples saved as list of lists
  - :issue_ghe:`265`: jekyll documentation
  - :issue_ghe:`277`: insert_self_transition_meta_data is never called
  - :issue_ghe:`268`: Enter key can still be used in greyed out window
  - :issue_ghe:`69`: Performance measurements
  - :issue_ghe:`271`: The storage folders are not always clean after re-saving a state machine from old format to new
  - :issue_ghe:`273`: Cannot refresh state machines
  - :issue_ghe:`264`: pylint under osl not working
  - :issue_ghe:`173`: Splash screen for RAFCON GUI initialization and RAFCON icon
  - :issue_ghe:`253`: Ctrl+V for pasting in list views of state editor does not work
  - :issue_ghe:`263`: The scrollbar in the io widget has to follow the currently edited text
  - :issue_ghe:`255`: After refreshing, state machines should keep their tab order
  - :issue_ghe:`185`: test_backward_stepping_barrier_state not working
  - :issue_ghe:`258`: Maximum recursion depth reached
  - :issue_ghe:`245`: Support library data port type change
  - :issue_ghe:`251`: Handles are added when hovering over a transition handle
  - :issue_ghe:`259`: Do not hard code version in about dialog
  - :issue_ghe:`260`: Meta data is loaded several times


Patch releases 0.8.\*
---------------------

0.8.4
"""""

- Improvements:
  - allow loading of state machines created with RAFCON 0.9.*


0.8.3
"""""

- Bug Fixes:
  - fix copy paste of library states, consisting of containers
  - fix error output of not matching output data types

0.8.2
"""""

- Bug Fixes:
  - fix copy and paste for ports
  - fix backward compatibility test

0.8.1
"""""

- Features:

  - renaming of module paths: core instead of state machine; gui instead of mvc
  - writing wrong data types into the outputs of the "execute" function produces an error now
  - Use external source editor: A button next to the source editor allows to open your code in an external editor, which you can configure
  - Gaphas: When resizing states, grid lines are shown helping states to bea aligned to each other (as when moving states)

- Improvements:

  - Gaphas: Change drawing order of state elements. Transitions are now drawn above states, Names of states are drawn
    beneath everything. This should ease the manipulation of transitions.
  - Gaphas: States are easier to resize, as the corresponding handle is easier to grab
  - states are now saved in folder that are named after: state.name + $ + state.state_id

- API:

  - library paths can now be defined relative to the config file (this was possible before, but only if the path was prepended with "./"

- Documentation:

  - started creation of "Developer's Guide"
  - moved ``odt`` document about commit guidelines into ``rst`` file for "Developer's Guide"

- Fixes:

  - :issue_ghe:`5`: Fix connection bug
  - :issue_ghe:`120`: Make state machines thread safe using RLocks
  - :issue_ghe:`154`: Multi-Selection problems
  - :issue_ghe:`159`: Transitions cannot be selected
  - :issue_ghe:`179`: Allow external source editor
  - :issue_ghe:`202`: RAFCON crash
  - :issue_ghe:`221`: issue when dragging data flows
  - :issue_ghe:`222`: Cannot remove transition of root state in TransitionController
  - :issue_ghe:`223`: rafcon library config relative path undefined behaviour
  - :issue_ghe:`224`: Switch to respective state when trying to open a state which is already open.

- Refactoring:

  - Widgets have TreeViews not have a common base class. This allowed to get rid of a lot of duplicate code and made some implementations more robust
  - the code behind connection creation and modification in the Gaphas editor has been completely rewritten and made more robust


0.8.0
"""""

- deactivated as not compatible with 0.7.13

Patch releases 0.7.\*
---------------------


0.7.13
""""""

- states are now saved in forlder that are named after: state.name + $ + state.state_id
- Hotfix:
  - fix unmovable windows for sled11 64bit

0.7.12
""""""

- Features:

  - Bidirectional graphical editor and states-editor selection with multi-selection support
  - Linkage overview widget redesign for optimized space usage and better interface

- Improvements:

  - Global variable manager and its type handling
  - Configuration GUI and its observation
  - State substitution: preserve default or runtime values of ports
  - Group/ungroup states
  - ``LibraryManager`` remembers missing ignored libraries
  - New config option ``LIBRARY_TREE_PATH_HUMAN_READABLE``: Replaces underscores with spaces in Library tree
  - Update of transition and data flow widgets

- API:

  - ``ExecutionHistory`` is now observable
  - Configurations are now observable
  - allow to set ``from_state_id`` id ``add_transition`` method for start transitions

- Fixes

  - :issue_ghe:`177`: Data flow hiding not working
  - :issue_ghe:`183`: Rafcon freeze after global variable delete
  - :issue_ghe:`53`: Configurations GUI
  - :issue_ghe:`181`: State type change not working
  - Several further fixes

- Refactorings, optimizations, clean ups


0.7.11
""""""

- Features:

  - Global variables can now be typed, see :issue_ghe:`Feature #81<81>`
  - GUI for modifying the configurations
  - Config files can be im- and exported
  - Graphical editor can be shown in fullscreen mode (default with
    F11), see :issue_ghe:`Feature #36<36>`
  - I18n: RAFCON can be translated into other languages, rudimentary
    German translation is available
  - RAFCON core can be started with several state machines

- Improvements:

  - Fix backward compatibility for old ``statemachine.yaml`` files
  - :issue_ghe:`136`: Undocked sidebars no longer have an entry in the task bar and are
    shown on top with the main window
  - Added tooltips
  - When starting RAFCON from the console, not only the path to, but
    also the file name of a config file can be specified. This allows
    several config files to be stored in one folder
  - Use correct last path in file/folder dialogs
  - Show root folder of libraries in the shortcut folder list of
    file/folder dialogs
  - new actions in menu bar, menu bar shows shortcuts
  - Source and description editor remember cursor positions

- API:

  - State machines and their models can be hashed

- Fixes

  - :issue_ghe:`161`: When refreshing a running state machine, the refreshed one is
    still running
  - :issue_ghe:`168`: Undocked sidebars cause issues with is\_focus()
  - :issue_ghe:`169`: Wrong dirty flag handling
  - :issue_ghe:`182`: Test start script waits infinitely
  - Several further fixes

- Refactorings, optimizations, clean ups

0.7.10
""""""

- Features

  - State substitution
  - Right click menu differentiate between states and library states

- Improvements

  - Graphical editor Gaphas:

  - way faster
  - more stable
  - connections are drawn behind states
  - small elements are hidden

  - BuildBot also runs tests on 32bit SLED slave
  - Core documentation

- Issues fixed

  - :issue_ghe:`143`
  - :issue_ghe:`139`
  - :issue_ghe:`146`
  - :issue_ghe:`145`
  - :issue_ghe:`122`
  - :issue_ghe:`149`
  - :issue_ghe:`119`
  - :issue_ghe:`151`
  - :issue_ghe:`155`
  - :issue_ghe:`17`

- Lots of further fixes and improvements

0.7.9
"""""

- Features:

  - Grouping and ungrouping of states
  - Initial version of possibility to save arbitrary states as
    libraries and to substitute one state with another one
  - Right click menu for graphical editor
  - add flags to ``mvc.start.py``

- Bug fixes

  - :issue_ghe:`132`
  - :issue_ghe:`40`
  - :issue_ghe:`65`
  - :issue_ghe:`131`
  - :issue_ghe:`105`
  - Kill RAFCON with Ctrl+C
  - Resizing of states in Gaphas
  - Correctly distinguish string and unicode data port types when using library states (should fix issues with ROS)
  - Stepping starts a state machine if not started

- Improvements

  - Gaphas works more reliable, especially concerning copy'n'paste and selection
  - History

- Some changes in destruction hooks
- Refactorings

  - Many for Gaphas components, e.g. the border size of a state depends on the state size now
  - Obsolete models are deleted (=> less memory consumption)
  - Remove state\_helper.py

- New network tests
- Add missing GUI drafts of Jürgen

0.7.8
"""""

- Add tests
- ExecutionEngine: Notify condition on all events except pause

0.7.7
"""""

- Add three new hooks

  - ``main_window_setup``: Passes reference to the main window controller and is called after the view has been registered
  - ``pre_main_window_destruction``: Passes reference to the main window controller and is called right before the main window is destroyed
  - ``post_main_window_destruction``: is called after the GTK main loop has been terminated

0.7.6
"""""

- remove obsolete files
- properly destruct states on their deletion (+ test to check unctionality)
- jump to state on double-click in ExecutionHistory
- fixes in display of ExecutionHistory
- fix not shown description of LibraryStates
- fix crash on middle-click on state machine tab
- Fix copy & paste of ExecutionStates
- improve tests
- improve documentation (add missing elements)
- Show '+' for adding state machines
- example on abortion handling
- Add config option to hide data flow name
- Fix :issue_ghe:`129`
- get rid of all plugin dependencies
- no more need to change into the mvc-directory when working with the GUI
- refactoring (especially in start.py)
- more fixes

0.7.5
"""""

- Improve Execution-History visualization with proper hierarchical tree
  view and improved data and logical outcome description (on
  right-click)
- Improve auto-backup and add lock files to offer formal procedure to
  recover state machine from temporary storage (see :ref:`Auto Recovery`)
- Improve Description editor by undo/redo feature similar to the
  SourceEditor
- Improve versions of "monitoring" and "execution hooks" plugins
- Improve graphical editor schemes (OpenGL and Gaphas) and Gaphas able
  to undo/redo state meta data changes
- Introduce optional profiler to check for computation leaks in state
  machine while execution
- Bug fixes

0.7.4
"""""

- Improve performance of GUI while executing state machine with high
  frequent state changes
- Fix :issue_ghe:`121`
  Properly copy nested ExecutionStates

0.7.3
"""""

- States are notified about pause and resume (See :ref:`FAQ` about :ref:`preemption <faq_preemption>` and
  :ref:`pause <faq_pause>`)
- Load libraries specified in
  :envvar:`RAFCON_LIBRARY_PATH` \(See :ref:`this tutorial <tutorial_libraries>`\)
- improve stability
- refactorings
- bug fixes

0.7.2
"""""

- improved auto-backup to tmp-folder
- fix missing logger messages while loading configuration files
- introduced templates to build plugins
- re-organized examples to one folder -> share/examples, with examples for API, libraries, plugins and tutorials
- introduce short-cut for applying ExecutionState-Scripts
- smaller bug fixes

0.7.1
"""""

- Allow multiple data flows to same input data ports (in order be
  remain backward compatibility)

0.7.0
"""""

This is a big minor release including many changes. State machines stored with version 0.6.\* are compatible with this version, but not state machines from older releases. Those have to be opened with 0.6.\* and then saved again. The following list is probably not complete:

- Support for openSUSE Leap
- Support for plugins
- Major design overhaul: agrees with drafts from design and looks consistent on all platforms
- Drag and Drop of states

  - Libraries from the library tree
  - Any type of state from the buttons below the graphical state editor
  - The drop position determines the location and the parent of the
    new state

- All sidebars can now be undocked and moved to another screen
- Auto store state machine in background and recover after crash
- Improved history with branches
- New feature: run until state
- Extended stepping mode: step into, over and out
- Redesign remote execution of state machines: Native GUI can be used to execute state machine running on different host
- Drop support of YAML state machine files
- Rename state machine files
- Extend documentation
- RMC-BuildBot support
- Many bug fixes
- A lot of refactorings, code optimizations, etc.


Patch releases 0.6.\*
---------------------

0.6.0
"""""

- Prepare code and folder structure to allow theming (currently only dark theme available)
- Refactor GUI configuration and color handling
- Fix network\_connection initialization
- Use python2.7 by default when using RAFCON with RMPM
- Gaphas graphical editor:

  - change cursor when hovering different parts of the state machine
  - add hover effect for ports
  - no more traces of states/labels when moving/resizing states/ports
  - resize handles are scaled depending on zoom level and state hierarchy
  - do not show handles on lines that cannot be moved
  - improve behavior of line splitting
  - refactorings
  - minor bug fixes

- Fix many code issues (line spacing, comments, unused imports, line length, ...)
- fix bug in global variable manager, causing casual exception when two threads access the same variable

Patch releases 0.5.\*
---------------------

0.5.5
"""""

fix start from selected state (the start-from-selected-state functionality modifies the start state of a hierarchy state on the initial execution of the statemachine; the start state was accidentally modified for each execution of the hierarchy state during one run leading to wrong execution of hierarchy states that were executed more often during the execution of a statemachine)

0.5.4
"""""

hotfix for mvc start.py launching with network support enabled

0.5.3
"""""

hotfix for rafcon server

0.5.1 + 0.5.2
"""""""""""""

feature: command line parameter to start state machine at an arbitrary state

0.5.0
"""""

- State-machines can be stored in JSON files instead of YAML files

  - Set USE\_JSON parameter in config to True
  - Loads state-machines approximately five times faster

- Removed some code ensuring backwards compatibility of old state-machines

  - If you are having trouble loading older state-machines, open them with the last version of the 0.4.\* branch
  - Save them and try again with the 0.5.\* branch

Patch releases 0.4.\*
---------------------

0.4.6
"""""

- Add start scripts in bin folder
- When using RAFCON with RMPM, you can run RAFCON just with the commands ``rafcon_start`` or ``rafcon_start_gui``
- Bug fixes for state type changes

0.4.5
"""""

- Feature: Add late load for libraries
- State type changes work now with Gaphas graphical editor
- Minor code refactorings

0.4.4
"""""

- Fix bug: changing the execution state of a statemachine does mark a statemachine as modified

0.4.3
"""""

- Fix bug: data port id generation
- Fix bug: runtime value handling

0.4.2
"""""

- Feature: runtime values

0.4.1
"""""

- Fix bug: resize of libraries when loading state machine
- Fix bug: error when adding data port to empty root state

0.4.0
"""""

- Show content of library states
- Keep library tree status when refreshing library
- Allow to easily navigate in table view of the GUI using the tab key
- Refactor logger (new handlers) and logger view
- Many refactorings for Gaphas graphical editor
- Introduce caching for Gaphas graphical editor => big speed up
- Require port names to be unique
- Highlight tab of running state machine
- Default values of library states can be set to be overwritten
- Improve dialogs
- make meta data observable
- many bug fixes
- clean code
- ...

Patch releases 0.3.\*
---------------------

0.3.7
"""""

- rafcon no-gui start script also supports BarrierConcurrency and PreemptiveConcurrencyStates

0.3.6
"""""

- bugfix if no runtime\_config existing

0.3.5
"""""

- rafcon\_server can be launched from command line
- network config can be passed as an argument on startup

0.3.4
"""""

- first version of rafcon server released

0.3.3
"""""

- state machines can be launched without GUI from the command line

0.3.2
"""""

- Extend and clean documentation (especially about MVC) and add it to the release
- Waypoints are moved with transition/data flows (OpenGL editor)
- data type of ports of libraries are updated in state machines when being changed in the library
- bug fix: error when moving waypoint
- bug fix: add new state, when no state is selected

0.3.1
"""""

- Support loading of old meta data
- bug fix: errors when removing connected outcome
- bug fix: network config not loaded
- code refactoring: remove old controllers, consistent naming of the rest

0.3.0
"""""

- RAFCON server to generate html/css/js files for remote viewer (inside browser)
- optimize workflow:

  - root state of new state machines is automatically selected
  - new states can directly be added with shortcuts, without using the mouse beforehand
  - A adds hierarchy state (A for execution states)

- support loading of state machines generated with the old editor in the new editor
- bug fixes for graphical editor using gaphas (especially concerning the state name)
- bug fixes for states editor

Patch releases 0.2.\*
---------------------

0.2.5
"""""

- update LN include script (use pipe\_include and RMPM)
- allow configuration of shortcuts
- distinguish between empty string and None for ports of type str
- bug fixes in GUI (start state)

0.2.4
"""""

- introduce env variables RAFCON\_PATH and RAFCON\_LIB\_PATH
- automatically set by RMPM

0.2.3
"""""

- use of seperate temp paths for different users

0.2.2
"""""

- Allow RAFCON to be started from arbitrary paths

0.2.1
"""""

- minor code refactoring
- RMPM release test

0.2.0
"""""

- First release version
- Tool was renamed to RAFCON
