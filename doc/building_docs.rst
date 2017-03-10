Building the documentation
==========================

The documentation is written in the reStructuredText markup language and resides in the `doc` folder.
It contains instructions to parse the source code to also include the documentation about the API.

If you want to build the documentation as HTML page, you have to run `sphinx`. This can either be done with the
provided PyCharm run configuration `Build Documentation` or manually in the console:

.. code:: bash

    $ cd /path/to/rafcon/repository
    $ sphinx-build -b html doc build_doc

This will build the documentation in the `build_doc` folder. Pass `-b pdf` to generate a PDF instead of a HTML page.

