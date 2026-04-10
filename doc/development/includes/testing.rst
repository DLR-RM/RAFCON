Running tests with ``tox``
""""""""""""""""""""""""""

The simplest and most reliable way of running the tests is using tox. If you have not installed tox, do so using

.. code:: bash

    $ pip install tox

Then, in the simplest case you just call tox in the root directory of the RAFCON repository:

.. code:: bash

    $ tox

This will run the following environments:

* ``py27``, ``py3[4-7]``: Runs the test using the according Python interpreter
* ``coverage``: Runs the tests using Python 2.7 with a coverage report
* ``docs``: Builds the documentation and verifies all links
* ``check``: Verifies the sdist and wheel file

Specific environments can be run with the ``-e`` option:

.. code:: bash

    $ tox -e 2.7,3.4
    $ tox -e docs

When running the tests (``py27``, ``py3[4-7]`` or ``coverage``), you can pass custom options to pytest by listing
them after ``tox [tox options] --``. The default pytest options are ``-vx -m "(core or gui or share_elements) and not
unstable"``.

.. code:: bash

    $ tox -e 2.7 -- -x -m "core"
    $ tox -- -s -k "test__destruct"

Tox creates a virtualenv for each environment, based on the dependencies defined in ``pyproject.toml`` and ``tox.ini``.
These are only generated on the first run of an environment. If the dependencies change, you need to tell tox to
recreate the environments using the ``-r/--recreate`` option:

.. code:: bash

    $ tox -re py2.7

The RAFCON tests are annotated with a number of markers, allowing you to select specific tests:

* ``core``, ``gui``, ``share_elements``, ``network``: Tests located in a folder with that name
* ``unstable``: GUI tests that do not run very reliable (e.g. because of the window manager)

Pytest allows you to select tests based on markers using the ``-m`` option. Markers can be combined using
``not``, ``and``, ``or`` and paranthesis:

.. code:: bash

    $ tox -e 2.7 -- -x -m "gui and not unstable"

Writing tests
"""""""""""""

RAFCON provides a lot of tests in the ``tests/`` folder. Many of these tests are integration tests, unit tests are
unfortunately often missing. If a test only uses imports from ``rafcon.core``, it is to be placed in ``tests/core/``,
otherwise in ``tests/gui/``.

RAFCON uses ``pytest`` as testing framework. It e.g. auto detects your test files starting with ``test_*``. Please have
a look at the documentation before writing tests: https://pytest.org/

GUI tests
^^^^^^^^^

When you want to write an integration test using the GUI, a custom fixture named ``gui`` is provided
(``tests/gui/conftest.py``). Simply add ``gui`` as parameter to your test (no import is required for tests residing
beneath ``test/gui/``). The fixture automatically starts the GUI before the test and closes it thereafter.

**Important:** Do not import any module from ``rafcon.gui`` outside of a function! Otherwise, models might be created
withing the wrong thread, which leads to ``gtkmvc`` forwarding observer notifications asynchronously (via ``idle_add``)
instead of synchronously.

When calling an operation that causes changes (in the core, models, GUI), you need to add the operation to the GTK queue
and wait until the operation has finished. This is simply done by calling ``gui(function_reference, *args, **kwargs)``
instead of ``function_reference(*args, **kwargs)``.

If your test commands causes any ``logger.warning`` or ``logger.error``, you need to specify the expected numbers. Do so
by calling ``gui.expected_warnings += 1``, respectively  ``gui.expected_errors += 1``, directly after the command that
causes the warning/error.

The fixture will load the default core and gui config options and the libraries ``generic`` and
``unit_test_state_machines``. If you want to override certain settings or add more libraries, use the following
decorator:

.. code-block:: python

    @pytest.mark.parametrize('gui', [{
        "gui_config": {
            'AUTO_BACKUP_ENABLED': True,
            'HISTORY_ENABLED': True
        },
        "libraries": {
            "ros": os.path.join(testing_utils.EXAMPLES_PATH, "libraries", "ros_libraries"),
            "turtle_libraries": os.path.join(testing_utils.EXAMPLES_PATH, "libraries", "turtle_libraries")
        }
    }], indirect=True, ids=["with history, auto backup, ros and turtle libraries"])
    def test_name(gui):
        pass  # test code

Using the ``ids`` argument, you can specify a label for your configuration. Other possible keys are ``core_config``
(``dict``), ``runtime_config`` (``dict``) and ``with_gui`` (``bool``, for tests that operate on models but do not
require the controllers and views). It is also possible to combine this with parameter sets:

.. code-block:: python

    config_options = {
        "gui_config": {
            'HISTORY_ENABLED': True
        }
    }
    @pytest.mark.parametrize("gui,state_path,recursive,rel_size", [
        (config_options, state_path_root, False, (40, 40)),
        (config_options, state_path_root, True, (40, 40)),
        (config_options, state_path_P, False, (20, 20)),
        (config_options, state_path_P, True, (20, 20)),
    ], indirect=["gui"])
    def test_name(gui, state_path, recursive, rel_size, monkeypatch):
        pass  # test code

Note that in this case, you need to set the ``indirect`` parameter to ``["gui"]``.

The ``gui`` fixture offers some features:

* if you want to restart the GUI *within* a test, call ``gui.restart()``
* the fixture provides shorthand access the gui singletons via ``gui.singletons`` and core singletons via
  ``gui.core_singletons``, without requiring any further imports.
* if you want to run a test *after* the GUI was closed, you can set the function to be run via
  ``gui.post_test = functools.partial(function_reference, *args, **kwargs)``

Tutorial - How to Write GUI Unit Tests for RAFCON
"""""""""""""""""""""""""""""""""""""""""""""""""

When you want to write a GUI unit test for RAFCON, you basically have two choices depending on what you are trying to test.

The first choice is a **GTK widget test**. This is the right pick when you want to test things like clicking buttons, toggling checkboxes, selecting items in a tree view, and generally anything where you are interacting with a specific GTK widget directly.

The second choice is a **GTK event simulation test**. This is the right pick when you are testing something that happens on the graphical canvas, like resizing a state, dragging something, or moving a handle. In this case you are not clicking a button widget, you are simulating the raw mouse events that GTK normally receives from the OS.

Both approaches use the same ``gui`` fixture and the same ``gui()`` wrapper, so they share a lot of common ground. The difference is just in how you interact with the UI.


The ``gui`` Fixture and the ``gui()`` Wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before jumping into examples, there is one thing you need to understand first, and that is the ``gui`` fixture.

RAFCON runs its GTK main loop in a separate thread. This means you cannot just call GTK functions directly from your test thread, because GTK is not thread safe and things will break or silently fail. Every time you want to do something to the GUI, you have to run it through the GTK thread.

That is exactly what the ``gui`` fixture gives you. When you do ``gui(some_function, arg1, arg2)``, it schedules ``some_function(arg1, arg2)`` to run in the GTK main thread, waits for it to finish, and returns the result. Simple as that.

Here is how you get it in your test:

.. code-block:: python

    # In your test file, just add `gui` as a parameter to your test function.
    # pytest will inject the fixture automatically.

    def test_something(gui):
        # `gui` is now your callable wrapper
        gui(some_gtk_widget.set_active, True)

If you want to configure things like window size or history settings, you can pass a config dict:

.. code-block:: python

    import pytest

    GUI_CONFIG = {
        "gui_config": {
            "HISTORY_ENABLED": True,
        },
        "runtime_config": {
            "MAIN_WINDOW_SIZE": (1500, 800),
            "MAIN_WINDOW_POS": (0, 0),
        }
    }

    @pytest.mark.parametrize("gui", [GUI_CONFIG], indirect=True, ids=["with fixed config"])
    def test_something(gui):
        ...


Choice 1: GTK Widget Tests
^^^^^^^^^^^^^^^^^^^^^^^^^^

This approach is for testing panels and widgets in the RAFCON UI. The breakpoints panel test is a good example of this.

The general flow is:

1. Load a state machine
2. Get the controller for the panel you want to test
3. Interact with the widgets on that controller's view
4. Check that the model or application state changed the way you expected

Example: The Breakpoints Panel Test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The test file is at ``tests/gui/test_breakpoints.py``. Let us walk through it step by step.

Step 1: Import what you need
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    import pytest
    import gui_singletons
    from rafcon.core import interface
    from rafcon.core.storage import storage
    from rafcon.core.singleton import state_machine_manager

``gui_singletons`` is where you find the main window controller and all the models after the GUI has started up. You will be using this a lot.

Step 2: Define the path to a test state machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    import os
    from tests import utils as testing_utils

    BOTTLES_PATH = os.path.join(testing_utils.TEST_ASSETS_PATH, "unit_test_state_machines", "99_bottles_of_beer_on_the_wall")

You need a real state machine to test with. RAFCON has a bunch of them under ``tests/assets/``.

Step 3: Load the state machine and open it in the GUI
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    def test_breakpoints(gui):
        sm = gui(storage.load_state_machine_from_path, BOTTLES_PATH)
        sm_id = gui(state_machine_manager.add_state_machine, sm)
        testing_utils.wait_for_gui()

Notice that ``gui(storage.load_state_machine_from_path, BOTTLES_PATH)`` is just calling ``storage.load_state_machine_from_path(BOTTLES_PATH)`` but doing it safely in the GTK thread.

``testing_utils.wait_for_gui()`` drains the GTK event queue after adding the state machine, giving the GUI time to finish rendering everything.

Step 4: Get references to what you need
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        sm_m = gui_singletons.state_machine_manager_model.state_machines[sm_id]
        breakpoints_ctrl = gui_singletons.main_window_controller.get_controller("breakpoints_ctrl")
        states_editor_ctrl = gui_singletons.main_window_controller.get_controller("states_editor_ctrl")

Here you are grabbing the model for your state machine and the controllers for the panels you want to interact with.

The ``get_controller("breakpoints_ctrl")`` call walks the controller tree and finds the breakpoints panel controller by its name. You can look up controller names by reading the view or controller source files, or by searching for ``register_view_callbacks`` in the controllers.

Step 5: Open a state in the states editor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        count_state = sm.root_state.states["mRODp"]
        count_state_m = sm_m.root_state.states["mRODp"]

        gui(states_editor_ctrl.activate_state_tab, count_state_m)
        testing_utils.wait_for_gui()

Some widgets only exist after their tab is opened. The breakpoint checkbox lives in the properties tab of a state, so you have to open that state first.

Step 6: Interact with the widget
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        count_tab_key = states_editor_ctrl.get_state_identifier(count_state_m)
        props_ctrl = states_editor_ctrl.tabs[count_tab_key]["controller"].get_controller("properties_ctrl")

        gui(props_ctrl.view["breakpoint_checkbox"].set_active, True)
        testing_utils.wait_for_gui()

This is the main interaction. You are getting the checkbox widget from the view and calling `set_active(True)` on it, which is exactly what would happen if a user clicked the checkbox.

The key pattern is: ``gui(widget.method, argument)``.

Step 7: Assert that the model changed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        from rafcon.core.singleton import breakpoint_manager as bm

        assert bm.should_pause(count_state)

After interacting with the widget, you check that the underlying model or application state is what you expect. The test does not care about the visual appearance, it cares about whether the logic worked.

Step 8: Test the panel controls
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

        toggle_btn = breakpoints_ctrl.view["toggle_all_button"]
        gui(toggle_btn.set_active, True)
        testing_utils.wait_for_gui()

        assert not bm.should_pause(count_state)

Same pattern again. Get the widget, call the method through ``gui()``, then assert.

To test the remove button, you select a row in the tree first, then call the handler:

.. code-block:: python

        gui(breakpoints_ctrl.breakpoints_tree.get_selection().select_path, 0)
        gui(breakpoints_ctrl.on_remove_selected, None)
        testing_utils.wait_for_gui()

        assert len(list(breakpoints_ctrl.breakpoints_store)) == 0


Choice 2: GTK Event Simulation Tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This approach is for testing things that happen on the graphical canvas. Instead of clicking a widget, you are creating raw GTK events like ``BUTTON_PRESS``, ``MOTION_NOTIFY``, and ``BUTTON_RELEASE``, and passing them directly to the tool or controller that would normally receive them.

The state resize test is a good example of this. The file is at ``tests/gui/test_state_resize.py``.

Example: The State Resize Test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The idea here is to simulate a user clicking and dragging the resize handle of a state on the canvas.

Step 1: Set up the state machine and get the canvas
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from gi.repository import Gdk
    from rafcon.gui.mygaphas.tools import MoveHandleTool

    def test_state_resize(gui, monkeypatch):
        # load and open the state machine (same as before)
        ...

        # get the graphical editor for the open state machine
        sm_m = gui_singletons.state_machine_manager_model.get_selected_state_machine_model()
        page = gui_singletons.main_window_controller.view["notebook"].get_nth_page(0)
        testing_utils.focus_graphical_editor_in_page(page)

Step 2: Use monkeypatch to make the test deterministic
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a real user drags a resize handle, GTK figures out which handle is under the cursor by checking pixel coordinates. In a test, pixel positions are hard to predict because the window may render slightly differently.

So instead of trying to hit the exact pixel, you monkeypatch the handle detection function to always return the handle you want:

.. code-block:: python

    from rafcon.gui.mygaphas import aspect
    from gaphas.geometry import Rectangle

    state_v = ...  # get the view for the state you want to resize

    def get_resize_handle(x, y, distance=None):
        return state_v, state_v.handles()[-1]  # always return the bottom-right handle

    monkeypatch.setattr(
        aspect.StateHandleFinder, "get_handle_at_point", get_resize_handle
    )

This way the test always resizes the correct state from the correct handle, regardless of window position.

Step 3: Create and send the events
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    resize_tool = MoveHandleTool(page.canvas)

    # Press down
    button_press = Gdk.Event.new(Gdk.EventType.BUTTON_PRESS)
    button_press.button = 1
    button_press.x = 300.0
    button_press.y = 300.0
    gui(resize_tool.on_button_press, button_press)

    # Move the mouse (simulate dragging)
    for i in range(10):
        motion = Gdk.Event.new(Gdk.EventType.MOTION_NOTIFY)
        motion.x = 300.0 + (i + 1) * 5
        motion.y = 300.0 + (i + 1) * 5
        gui(resize_tool.on_motion_notify, motion)
        testing_utils.wait_for_gui()

    # Release
    button_release = Gdk.Event.new(Gdk.EventType.BUTTON_RELEASE)
    button_release.button = 1
    gui(resize_tool.on_button_release, button_release)
    testing_utils.wait_for_gui()

The idea is you are playing the role of the OS. Normally the OS sends these events to GTK and GTK forwards them to the tool. Here you are just cutting out the middleman and sending them directly to the tool yourself.

Step 4: Assert on the state size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    new_size = (state_v.model.state.width, state_v.model.state.height)
    assert new_size[0] > original_size[0]
    assert new_size[1] > original_size[1]


Utility Functions Reference
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Run anything in the GTK thread: ``gui(function, arg1, arg2)``
- Wait for GTK to finish processing: ``testing_utils.wait_for_gui()``
- Give focus to the graphical canvas: ``testing_utils.focus_graphical_editor_in_page(page)``
- Get any controller by name: ``gui_singletons.main_window_controller.get_controller(...)``
- Get selected state machine model: ``gui_singletons.state_machine_manager_model.get_selected_state_machine_model()``
- Load a state machine from disk: ``gui(storage.load_state_machine_from_path, path)``

Quick Decision Guide
^^^^^^^^^^^^^^^^^^^^

Use the **GTK widget approach** when you are testing:

- A panel with buttons, checkboxes, labels, or tree views
- The properties editor for a state
- The breakpoints panel, the execution history panel, and similar side panels

Use the **GTK event simulation approach** when you are testing:

- Something that happens directly on the graphical canvas
- Resizing, moving, or connecting states
- Drag and drop operations on the canvas

If you are not sure, ask yourself: is there a GTK widget I can grab and call a method on? If yes, use the widget approach. If you are dealing with the canvas, use the event simulation approach.