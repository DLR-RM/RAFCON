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
