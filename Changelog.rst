Patch releases 0.7.\*
=====================

0.7.12
------

- Improvements:

   - Global variable manager and its type handling
   - Configuration GUI and its observation
   - State substitution: preserve default or runtime values of ports
   - Group/ungroup states
   - ``LibraryManager`` remembers missing ignored libraries
   - New config option ``LIBRARY_TREE_PATH_HUMAN_READABLE``: Replaces underscores with spaces in Library tree

-  API:

   - ``ExecutionHistory`` is now observable
   - Configurations are now observable
   - allow to set `from_state_id` id `add_transition` method for start transitions

-  Fixes

   -  `#177 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/177>`__:
      Data flow hiding not working
   -  `#183 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/183>`__:
      Rafcon freeze after global variable delete
   -  `#53 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/53>`__:
      Configurations GUI
   -  `#181 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/181>`__:
      State type change not working
   -  Several further fixes

-  Refactorings, optimizations, clean ups


0.7.11
------

-  Features:

   -  Global variables can now be typed, see issue
      `#81 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/81>`__
   -  GUI for modifying the configurations
   -  Config files can be im- and exported
   -  Graphical editor can be shown in fullscreen mode (default with
      F11), see issue
      `#36 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/36>`__
   -  I18n: RAFCON can be translated into other languages, rudimentary
      German translation is available
   -  RAFCON core can be started with several state machines

-  Improvements:

   -  Fix backward compatibility for old ``statemachine.yaml`` files
   -  Undocked sidebars no longer have an entry in the task bar and are
      shown on top with the main window, see issue
      `#136 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/136>`__
   -  Added tooltips
   -  When starting RAFCON from the console, not only the path to, but
      also the file name of a config file can be specified. This allows
      several config files to be stored in one folder
   -  Use correct last path in file/folder dialogs
   -  Show root folder of libraries in the shortcut folder list of
      file/folder dialogs
   -  new actions in menu bar, menu bar shows shortcuts
   -  Source and description editor remember cursor positions

-  API:

   -  State machines and their models can be hashed

-  Fixes

   -  `#161 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/161>`__:
      When refreshing a running state machine, the refreshed one is
      still running
   -  `#168 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/168>`__:
      Undocked sidebars cause issues with is\_focus()
   -  `#169 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/169>`__:
      Wrong dirty flag handling
   -  `#182 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/182>`__:
      Test start script waits infinitely
   -  Several further fixes

-  Refactorings, optimizations, clean ups

0.7.10
------

-  Features

   -  State substitution
   -  Right click menu differentiate between states and library states

-  Improvements

   -  Graphical editor Gaphas:

      -  way faster
      -  more stable
      -  connections are drawn behind states
      -  small elements are hidden

   -  BuildBot also runs tests on 32bit SLED slave
   -  Core documentation

-  Issues fixed

   -  `Issue
      #143 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/143>`__
   -  `Issue
      #139 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/139>`__
   -  `Issue
      #146 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/146>`__
   -  `Issue
      #145 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/145>`__
   -  `Issue
      #122 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/122>`__
   -  `Issue
      #149 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/149>`__
   -  `Issue
      #119 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/119>`__
   -  `Issue
      #151 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/151>`__
   -  `Issue
      #155 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/155>`__
   -  `Issue
      #17 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/155>`__

-  Lots of further fixes and improvements

0.7.9
-----

-  Features:

   -  Grouping and ungrouping of states
   -  Initial version of possibility to save arbitrary states as
      libraries and to substitute one state with another one
   -  Right click menu for graphical editor
   -  add flags to ``mvc.start.py`` `\(see commit 87e8cd7\) <https://rmc-github.robotic.dlr.de/common/rafcon/commit/87e8cd7e64648aea8255db7b191112624a210c94>`__

-  Bug fixes

   -  `Issue
      #132 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/132>`__
   -  `Issue
      #40 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/40>`__
   -  `Issue
      #65 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/65>`__
   -  `Issue
      #131 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/40>`__
   -  `Issue
      #105 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/105>`__
   -  Kill RAFCON with Ctrl+C
   -  Resizing of states in Gaphas
   -  Correctly distinguish string and unicode data port types when
      using library states (should fix issues with ROS)
   -  Stepping starts a state machine if not started

-  Improvements

   -  Gaphas works more reliable, especially concerning copy'n'paste and
      selection
   -  History

-  Some changes in destruction hooks
-  Refactorings

   -  Many for Gaphas components, e.g. the border size of a state
      depends on the state size now
   -  Obsolete models are deleted (=> less memory consumption)
   -  Remove state\_helper.py

-  New network tests
-  Add missing GUI drafts of Jürgen

0.7.8
-----

-  Add tests
-  ExecutionEngine: Notify condition on all events except pause

0.7.7
-----

-  Add three new hooks

   -  ``main_window_setup``: Passes reference to the main window
      controller and is called after the view has been registered
   -  ``pre_main_window_destruction``: Passes reference to the main
      window controller and is called right before the main window is
      destroyed
   -  ``post_main_window_destruction``: is called after the GTK main
      loop has been terminated

0.7.6
-----

-  remove obsolete files
-  properly destruct states on their deletion (+ test to check
   functionality)
-  jump to state on double-click in ExecutionHistory
-  fixes in display of ExecutionHistory
-  fix not shown description of LibraryStates
-  fix crash on middle-click on state machine tab
-  Fix copy & paste of ExecutionStates
-  improve tests
-  improve documentation (add missing elements)
-  Show '+' for adding state machines
-  example on abortion handling
-  Add config option to hide data flow name
-  `Fix Issue #129 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/129>`__
-  get rid of all plugin dependencies
-  no more need to change into the mvc-directory when working with the
   GUI
-  refactoring (especially in start.py)
-  more fixes

0.7.5
-----

-  Improve Execution-History visualization with proper hierarchical tree
   view and improved data and logical outcome description (on
   right-click)
-  Improve auto-backup and add lock files to offer formal procedure to
   recover state machine from temporary storage `Auto
   Recovery <https://rmintra01.robotic.dlr.de/wiki/RAFCON#Auto_Backup>`__
-  Improve Description editor by undo/redo feature similar to the
   SourceEditor
-  Improve versions of "monitoring" and "execution hooks" plugins
-  Improve graphical editor schemes (OpenGL and Gaphas) and Gaphas able
   to undo/redo state meta data changes
-  Introduce optional profiler to check for computation leaks in state
   machine while execution
-  Bug fixes

0.7.4
-----

-  Improve performance of GUI while executing state machine with high
   frequent state changes
-  Fix `issue
   121 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/121>`__:
   Properly copy nested ExecutionStates

0.7.3
-----

-  States are notified about pause and resume (See FAQ
   `here <https://rmintra01.robotic.dlr.de/wiki/RAFCON/FAQ#How_does_preemption_work.3F_How_do_I_implement_preemptable_states_correctly.3F>`__
   and
   `here <https://rmintra01.robotic.dlr.de/wiki/RAFCON/FAQ#What_happens_if_the_state_machine_is_paused.3F_How_can_I_pause_running_services.2C_e._g._the_robot.3F>`__)
-  Load libraries specified in
   ``RAFCON_LIBRARY_PATH`` \(See `this tutorial <https://rmintra01.robotic.dlr.de/wiki/RAFCON/Tutorials#How_to_create_and_re-use_a_library_state_machine>`__\)
-  improve stability
-  refactorings
-  bug fixes

0.7.2
-----

-  improved auto-backup to tmp-folder
-  fix missing logger messages while loading configuration files
-  introduced templates to build plugins
-  re-organized examples to one folder -> share/examples, with examples
   for API, libraries, plugins and tutorials
-  introduce short-cut for applying ExecutionState-Scripts
-  smaller bug fixes

0.7.1
-----

-  Allow multiple data flows to same input data ports (in order be
   remain backward compatibility)

0.7.0
-----

This is a big minor release including many changes. State machines
stored with version 0.6.\* are compatible with this version, but not
state machines from older releases. Those have to be opened with 0.6.\*
and then saved again. The following list is probably not complete:

-  Support for `openSUSE Leap <https://rmintra01.robotic.dlr.de/wiki/OpenSUSE_Leap>`__
-  Support for plugins
-  Major design overhaul: agrees with drafts from design and looks
   consistent on all platforms
-  Drag and Drop of states

   -  Libraries from the library tree
   -  Any type of state from the buttons below the graphical state
      editor
   -  The drop position determines the location and the parent of the
      new state

-  All sidebars can now be undocked and moved to another screen
-  Auto store state machine in background and recover after crash
-  Improved history with branches
-  New feature: run until state
-  Extended stepping mode: step into, over and out
-  Redesign remote execution of state machines: Native GUI can be used
   to execute state machine running on different host
-  Drop support of YAML state machine files
-  Rename state machine files
-  Extend documentation
-  `RMC-BuildBot <https://rmintra01.robotic.dlr.de/wiki/Buildbot>`__ support
-  Many bug fixes
-  A lot of refactorings, code optimizations, etc.
