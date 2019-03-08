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
* ``check``: Verifies the sdist file

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

Tox creates a virtualenv for each environment, based on the dependencies defined in ``setup.py`` and ``tox.ini``.
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
