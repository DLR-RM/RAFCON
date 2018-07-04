Changelog
=========

Information about :ref:`RAFCON` changes in each release will be published here. More
details can be found in the `GIT commit log <https://github.com/DLR-RM/RAFCON/commits/develop>`__.

Next release
------------

- Features:

- Improvements:

- Bug Fixes:


Patch releases 0.12.\*
----------------------

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

    - :issue:`530` automatically focus and adapt position of root state for fresh initiated state machines
                   issue title was "Root state out of focus and badly positioned"
    - :issue:`543` Changing default option for library name while saving
                   -> for the default folder name white space are replaced with underscores and all is lower case
    - also default library state name is now the folder name with replaced underscores with white spaces


- Bug Fixes:

    - :issue:`527` RAFCON GUI loops while startup if HOME environment variable is not defined
                   -> a error message pointing on respective missing environment variable is added
    - :issue:`539` grouping of states outcome transitions are not fully recovers (now bug is covered by test)
    - :issue:`515` source editor does not show end of lines (finally)


0.12.11
"""""""

- Improvements:

    - :issue:`529` accelerate the follow mode switch for many logger messages
    - dynamic insertion of states during state execution is working and tested
    - secure dynamic modification of state machines while runtime by test created in
      pull request :issue:`535` Dynamic insertion of states during execution

- Bug Fixes:

    - :issue:`515` source editor does not show end of lines (partly)
    - :issue:`533` States inside library states cannot be selected
    - :issue:`528` execution history destruction does not lead to max recursion depth


0.12.10
"""""""

- Features:

    - :issue:`520` Debug Console keeps track of last logger message if the follow mode is enabled

- Improvements:

    - in pull request :issue:`523` refactoring of debug console  for more intuitive and robust behavior
      e.g. persistent cursor position
    - :issue:`516` source editor does not show line of cursor after apply if the script is big

- Bug Fixes:

    - :issue:`519` rafcon freezes while opening a state machine
        - solved in pull request :issue:`524` history elements hold direct state reference
    - :issue:`514` text in entry widget of port not visible during editing (arrow key press left-right helps)
        - the issue was not fully resolved but improved

0.12.9
""""""

- Improvements:

    - container state API can adjust output_data by new method write_output_data
    - more robust execution history tree
    - performance improvement by deleting gaphas views at once for recursive state destruction's

- Bug Fixes:

    - :issue:`521` Strange gaphas logs during deletion of a state
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

    - :issue:`503` scoped variable looks weird
    - :issue:`505` clean up profiler flag in config
    - :issue:`506` root state input ports leave ugly stripes behind
    - :issue:`501` transition is not selectable if it is drawn over state
    - :issue:`512` execution of second state machine cause freeze of stop on previous state machine was not successful
    - :issue:`514` text in entry widget of port not visible during editing
    - fix state machine tree remove library state
    - no deadlocks when locking a global variable two times
    - :issue:`502` changing data ports not possible
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

    - :issue:`484` label handles are hard to grasp
    - :issue:`486` Gaphas is not emitting meta data signal if NameView is moved
    - quick fix for not working "state type change" in combination with library states (which was based on respective
      object destruction while those operations) -> will be fully solved in :issue:`493`
    - quick fix for not set or too late set of active state machine id -> will be fully solved in :issue:`495`
    - fix meta data for undo/redo of add object operations
    - fix exception handling, causing issues with the graphical editor when invalid connection were created
    - When hovering the menu bar, an exception was printed


0.12.4
""""""

- Improvements:

    - Provide a `PULL_REQUEST_TEMPLATE` for pull requests opened in GitHub
    - Optimize updates/redrawing of graphical editor

- Bug Fixes:

    - :issue:`414` state machines with libraries cannot be closed


0.12.3
""""""

- Feature
    - The env variable :envvar:`RAFCON_START_MINIMIZED` allows to start RAFCON minimized, which is helpful when running
      the tests

- Improvements:

    - :issue:`414` Memory optimizations: The memory usage should no longer increase over time, as unused objects are now freed
    - A new/extended test verifies the correct destruction of removed elements
    - Optimize NameView font size calculations, noticeable during zooming
    - ports outside of the visible view are no longer drawn, which increases the performance, especially while
      zooming in large state machines
    - Hash calculations of state machines
    - Placement of NameView
    - drawing of connections, ports and labels, especially when deeply nested
    - :issue:`469` unit test refactorings

- Bug Fixes:

    - :issue:`459` execution_log utils; backward compatibility missing and :issue:`458` ReturnItem
    - :issue:`454` group/ungroup is not preserving meta data recursively
    - :issue:`452` Session restore, gaphas and extended controller causes exception when closing RAFCON
    - :issue:`450` Names of states inside a library become smaller
    - :issue:`447` Hashes of state machine in storage different then the reopened state machine after saving it
    - :issue:`449` ports (of transitions or data flows) cannot be moved
    - :issue:`471` selection of states in hierarchies >= 5 not possible


0.12.2
""""""

- New Features:

    - Fix logging for library state execution

- Improvements:

    - Improve execution logging (semantic data is supported now)
    - :issue:`445` Tests need to ensure correct import order for GUI singletons

- Bug Fixes:

    - :issue:`446` "show content" leads to sm marked as modified


0.12.1
""""""

- New Features:

    - Semantic data editor supports external editor
    - Transparency of library states improved when content is shown

- Improvements:

    - :issue:`415` Increase visibility of library content

- Bug Fixes:

    - :issue:`378` Editing default values does not work sometimes


0.12.0
""""""

- New Features:

    - Semantic meta data editor and storage for every state
    - :issue:`411` Allow outputting data from preempted states

- Bug Fixes:

    - :issue:`426` Again meta data of library ports are screwed after insertion
    - :issue:`425` Connection via points not visible
    - :issue:`424` Wrong path for tooltip for state machines editor tabs
    - :issue:`431` Test for recently opened state machine fails
    - :issue:`430` Selection test fails



Patch releases 0.11.\*
----------------------

0.11.6
""""""

- Bug Fixes:

    - :issue:`428` fix recursion problem in execution log viewer
    - :issue:`427` Middle click on state machine tab label close wrong state machine
    - :issue:`419` wrong outcome data in execution history

- Improvements:

    - :issue:`411` Allow outputting data from preempted states
    - drag'n drop with focus can be enabled and disabled by using the gui config flag DRAG_N_DROP_WITH_FOCUS
    - graphical editor add way points around the state for self transitions as support for the user
    - refactor state machines editor tab click methods and small fixing
    - better on double click focus by gaphas editor and now also triggered by state machine tree

0.11.5
""""""

- Bug Fixes:
    - :issue:`421` RAFCON does not remember window size after closing -> final part

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
    - :issue:`411` Allow outputting data from preempted states
    - :issue:`410` Refactor selection
    - :issue:`403` Incomes and outcomes cannot be differentiated visually

- Bug Fixes:

    - Memory leak fixes
    - :issue:`402` Connections end in nowhere
    - :issue:`417` ports of root state do not move with roots state
    - :issue:`421` RAFCON does not remeber window size after closing -> first part

0.11.3
""""""

- Improvements:

    - :issue:`405` Possibility to zoom in and out while drawing a connection
    - :issue:`404` Possibility to scroll left and right in graphical editor
    - :issue:`403` Incomes and outcomes cannot be differentiated visually

- Bug Fixes:

    - :issue:`412` global variables cannot be removed
    - :issue:`413` tree view controller error

0.11.2
""""""

- Improvements:

    - meta data scaling more robust and protect other elements from side effects of it

- Bug Fixes:

    - :issue:`393` $HOME/.config/rafcon is not generated initially + tests
    - :issue:`406` Empty library root state without child states cause meta data resize problems with side effects in
      gaphas drawing

0.11.1
""""""

- New Features:

    - :issue:`384` add "Collapse all" button for library manager and enable the feature for the state machine tree, too

- Improvements:

    - port position default values

- Bug Fixes:

    - Fix issues when copying/converting logical or data ports with clipboard while cut/copy/paste
    - Fix library state port position scaling after adding
    - Fix gaphas viewer problems with undo/redo of complex actions like copy and paste or add/remove of ports
    - :issue:`10` Fully integrate modification history with gaphas

0.11.0
""""""

- New Features:

  - "Session restore" by default enabled
  - :issue:`364` "Open Recent" recently opened state state machines sub menu in menu bar under sub-menu Files
  - "Save as copy" in menu bar under sub-menu Files
  - "Show library content" supported for gaphas graphical viewer
  - The inner library states can be selected, copied and used to run the execution from or to this state,
    see :issue:`366` and :issue:`367`, too
  - :issue:`255` The state machine tree shows inner library states, too, and can be used to explore all "leaf"-states
  - Storage format can be adapted by the user (e.g. names of states in paths and there length)
  - The library manager widget/tree supports modifications by right click (remove library, add/remove library roots)
  - Execution tool-bar supports buttons for run to- and run from-state (like right click menu, too)

- Improvements:

  - Refactoring of "Save state as state machine/library"
  - Better default position meta data for states in graphical viewer
  - Proper resize of graphical meta data for complex actions and show library content
  - :issue:`369` Storage/Load module for state machines more flexible and robust
  - Storage module supports the user to store state machines without platform specific file system format conflicts
  - :issue:`365` substitute widget in now scrollable
  - The gtkmvc version 1.99.2 is fully supported (:issue:`388` corrected version in older releases)

- Bug Fixes:

  :issue:`382` Currently active state machine not correct
  :issue:`362` Data flows between scoped variables
  :issue:`354` Meta data broken when adding state as template to state machine
  :issue:`353` Label not shown when adding state from library

Patch releases 0.10.\*
----------------------

0.10.3
""""""

- Bug Fixes:

  - File Chooser crashed if the same folder was added to the shortcut_folders twice

0.10.2
""""""

- Bug Fixes:

  - :issue:`385` If runtime config is newly created the last open path is empty and now state machine could be saved

0.10.1
""""""

- Bug Fixes:
  
  - make execution logs compatible with execution log viewer again


0.10.0
""""""

- Improvements:
  
  - complex actions(copy & paste, resize) are properly handled in gaphas and in the modification history
  - :issue:`342` drag and drop now drops the state at the mouse position

- Bug Fixes:
  
  - show library content for OpenGL works again  
  - add as template works again
  - :issue:`343` Text field does not follow cursor

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
  - :issue:`336`: Use custom popup menu in state machine editor to quickly navigate in open state machines

- Bug Fixes

  - :issue:`349` Save as library functionality erroneous
  - :issue:`314` Recursion limit reached when including top statemachine as replacement for missing state machine
  - :issue:`341` Reload only selected state machine
  - :issue:`339` Only save the statemachine.json
  - :issue:`338` Selecting a library state should show the data ports widget per default
  - :issue:`327` State machines are not properly selected
  - :issue:`337` Pressing the right arrow in the state machine editor opens a new state machine
  - :issue:`346` Barrier State cannot be deleted

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
  - :issue:`322` Group/Ungroup is not working when performed on childs of a BarrierConcurrencyState
  - :issue:`326` RAFCON_INSTANCE_LOCK_FILE exception

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

  - :issue:`299`: State labels can be placed outside the state borders
  - :issue:`298`: Child states can be placed outside hierarchy states
  - :issue:`45`: Size of GUI cannot be changed
  - :issue:`284`: Core does not check the type of the default values
  - :issue:`282`: Input and output data port default_value check does not cover all cases
  - :issue:`280`: List of tuples saved as list of lists
  - :issue:`265`: jekyll documentation
  - :issue:`277`: insert_self_transition_meta_data is never called
  - :issue:`268`: Enter key can still be used in greyed out window
  - :issue:`69`: Performance measurements
  - :issue:`271`: The storage folders are not always clean after re-saving a state machine from old format to new
  - :issue:`273`: Cannot refresh state machines
  - :issue:`264`: pylint under osl not working
  - :issue:`173`: Splash screen for RAFCON GUI initialization and RAFCON icon
  - :issue:`253`: Ctrl+V for pasting in list views of state editor does not work
  - :issue:`263`: The scrollbar in the io widget has to follow the currently edited text
  - :issue:`255`: After refreshing, state machines should keep their tab order
  - :issue:`185`: test_backward_stepping_barrier_state not working
  - :issue:`258`: Maximum recursion depth reached
  - :issue:`245`: Support library data port type change
  - :issue:`251`: Handles are added when hovering over a transition handle
  - :issue:`259`: Do not hard code version in about dialog
  - :issue:`260`: Meta data is loaded several times
  

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

  - :issue:`5`: Fix connection bug
  - :issue:`120`: Make state machines thread safe using RLocks
  - :issue:`154`: Multi-Selection problems
  - :issue:`159`: Transitions cannot be selected
  - :issue:`179`: Allow external source editor
  - :issue:`202`: RAFCON crash
  - :issue:`221`: issue when dragging data flows
  - :issue:`222`: Cannot remove transition of root state in TransitionController
  - :issue:`223`: rafcon library config relative path undefined behaviour
  - :issue:`224`: Switch to respective state when trying to open a state which is already open.

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

  - :issue:`177`: Data flow hiding not working
  - :issue:`183`: Rafcon freeze after global variable delete
  - :issue:`53`: Configurations GUI
  - :issue:`181`: State type change not working
  - Several further fixes

- Refactorings, optimizations, clean ups


0.7.11
""""""

- Features:

  - Global variables can now be typed, see :issue:`Feature #81<81>`
  - GUI for modifying the configurations
  - Config files can be im- and exported
  - Graphical editor can be shown in fullscreen mode (default with
    F11), see :issue:`Feature #36<36>`
  - I18n: RAFCON can be translated into other languages, rudimentary
    German translation is available
  - RAFCON core can be started with several state machines

- Improvements:

  - Fix backward compatibility for old ``statemachine.yaml`` files
  - :issue:`136`: Undocked sidebars no longer have an entry in the task bar and are
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

  - :issue:`161`: When refreshing a running state machine, the refreshed one is
    still running
  - :issue:`168`: Undocked sidebars cause issues with is\_focus()
  - :issue:`169`: Wrong dirty flag handling
  - :issue:`182`: Test start script waits infinitely
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

  - :issue:`143`
  - :issue:`139`
  - :issue:`146`
  - :issue:`145`
  - :issue:`122`
  - :issue:`149`
  - :issue:`119`
  - :issue:`151`
  - :issue:`155`
  - :issue:`17`

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

  - :issue:`132`
  - :issue:`40`
  - :issue:`65`
  - :issue:`131`
  - :issue:`105`
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
- Fix :issue:`129`
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
- Fix :issue:`121`
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
