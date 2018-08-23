
RAFCON
======

.. image:: https://raw.githubusercontent.com/DLR-RM/RAFCON/master/documents/assets/Screenshot_Drill_Skill.png
   :width: 200px
   :align: left
   :alt: Screenshot showing RAFCON with a big state machine
   :target: documents/assets/Screenshot_Drill_Skill.png?raw=true


* Documentation: Hosted on `Read the Docs <http://rafcon.readthedocs.io/en/latest/>`_
* Homepage: `DLR-RM.github.io/RAFCON/ <https://dlr-rm.github.io/RAFCON/>`_
* License: `EPL <https://github.com/DLR-RM/RAFCON/blob/master/LICENSE>`_

Develop your robotic tasks using an intuitive graphical user interface
----------------------------------------------------------------------

RAFCON uses hierarchical state machines, featuring concurrent state execution, to represent robot programs.
It ships with a graphical user interface supporting the creation of state machines and
contains IDE like debugging mechanisms. Alternatively, state machines can programmatically be generated
using RAFCON's API.

Universal application

  RAFCON is written in Python, can be extended with plugins and is hard- and middleware independent.

Visual programming

  The sophisticated graphical editor can be used for the creation, execution and debugging of state machines.

Collaborative working

  Share and reuse your state machines in form of libraries, stored as JSON strings in text files.


Installation preparations
-------------------------

Before installing RAFCON, Python 2.7, pip and setuptools are required on your system. Most of the other dependencies
are automatically resolved by pip/setuptools, but not all of them. Those need be be installed manually, too:

Installation requirements for Ubuntu 16.04
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt install python-setuptools python-dev
   sudo apt install python-opengl python-gtkglext1 python-gtksourceview2

Installation requirements for Ubuntu 18.04
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo apt-get install python-setuptools python-dev build-essential python-opengl python-gtkglext1 python-gtksourceview2 python-pip
   pip2 install --user pylint


General requirements
^^^^^^^^^^^^^^^^^^^^

If you are not using Ubuntu 16.04 or 18.04, please make sure that the following packages are installed:

* Python 2.7
* python-setuptools
* pip
* python-opengl
* python-gtkglext1
* python-gtksourceview2
* pylint


Installing RAFCON (from PyPi)
-----------------------------

.. code-block:: bash

   pip2.7 install rafcon --user

The ``--user`` flag is optional. If not set, RAFCON is installed globally.


Download RAFCON sources
-----------------------

.. code-block:: bash

   cd /install/directory
   git clone https://github.com/DLR-RM/RAFCON rafcon


Installing RAFCON (non-editable from source)
--------------------------------------------

If you don't want to edit the source code of RAFCON, it can be installed directly from source:

.. code-block:: bash

   pip2.7 install /install/directory/rafcon/ --user


Installing RAFCON (editable from source)
----------------------------------------

If you want to be able to change the source code, you can install RAFCON in editable mode.

.. code-block:: bash

   pip2.7 install --editable /install/directory/rafcon/ --user

Any changes in ``/install/directory/rafcon/source`` will take effect when launching RAFCON.


Start RAFCON
------------

No matter which installation option you choose, RAFCON can be started from any location using (make sure ``~/.local/bin`` is in your ``PATH`` environment variable):

.. code-block:: bash

   rafcon


Building the documentation
--------------------------

The documentation is build with sphinx:

.. code-block:: bash

   sphinx-build -b html /install/directory/rafcon/doc /install/directory/rafcon/build_doc

This will build the documentation in the /install/directory/rafcon/build_doc folder. Pass ``-b pdf`` to generate a PDF instead of a HTML page.


Uninstallation
--------------

If you want to uninstall RAFCON, all you need to do is call

.. code-block:: bash

   pip2.7 uninstall rafcon


Clean RAFCON directory
----------------------

If you want to clean the RAFCON directory /install/directory/rafcon from any build/installation artifacts, you can do so with:

.. code-block:: bash

   cd /install/directory/rafcon
   rm -r build/ build_doc/ .eggs/ .cache/

