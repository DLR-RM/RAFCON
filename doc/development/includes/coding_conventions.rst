Style guides
""""""""""""

We follow the rules :pep:`8` (Style Guide for Python Code) and the :download:`DLR Coding Conventions
<../_static/code_conventions_python_flyer.pdf>` with some minor exceptions:

- Line length is limited to 120 characters (instead of 79)
- no ``__version__`` in header (except ``rafcon/__init__.py``)

For docstrings (and generally for the documentation), we format using `reStructuredText <http://docutils.sourceforge
.net/rst.html>`__ for `Sphinx <http://sphinx-doc.org/>`__. More information on this is available in another :pep:`287`.
 

PyCharm Coding Style/Conventions
""""""""""""""""""""""""""""""""

Our recommended tool for development is PyCharm. This IDE makes it easy for you to follow our style guides. We
prepared a settings file for "Code styles" and "Inspection profiles", which you just need to import: `File > Import
Settings > [project root/.idea/recommended_code_style_and_inspections.jar`.
