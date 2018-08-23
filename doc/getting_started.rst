Getting started
===============

For an introduction of how to install rafcon, have a look here: https://dlr-rm.github.io/RAFCON/getting_started.html

Also, to get the correct dependencies, follow the instruction given on this site.

The following describes how to get the latest RAFCON version per github.

First, change to the directory in which you want to clone RAFCON:

.. code:: bash

    $ cd /some/personal/path/

Next, clone the `RAFCON
repository <https://github.com/DLR-RM/RAFCON>`__. You can
either use the HTTPS URL:

.. code:: bash

    $ git clone https://github.com/DLR-RM/RAFCON.git

or the SSH URL:

.. code:: bash

    $ git clone git@github.com:DLR-RM/RAFCON.git

This must of course only be done once. If you want to get the latest
commits after you have cloned the repository, use

.. code:: bash

    $ cd /some/personal/path/rafcon
    $ git pull

In order to run RAFCON from the local code base, you have to setup the
environment:

.. code:: bash

    $ export PYTHONPATH=/some/personal/path/rafcon/source:$PYTHONPATH
    $ export PATH=/some/personal/path/rafcon/bin:$PATH

Now you can run ``rafcon`` to start the RAFCON-GUI or just run ``rafcon_core`` to only launch the core. Hereby,
``rafcon`` just links to the file ``/some/personal/path/rafcon/source/rafcon/gui/start.py`` and ``rafcon_core``
points to ``/some/personal/path/rafcon/source/rafcon/core/start.py``, so you could also call these files directly.

.. _install_fonts:

**IMPORTANT:** If you start rafcon for the first time start it this way:

.. code:: bash

    $ RAFCON_CHECK_INSTALLATION=True rafcon

This will install all fonts and gtk-styles.
