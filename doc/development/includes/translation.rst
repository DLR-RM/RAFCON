
RAFCON is prepared for internationalization (i18n) and in fact ships with a basic German translation.
This translation can be extended and new translations can be added. We are happy about your support here!


Add translatable strings
""""""""""""""""""""""""

In order to allow for translating strings in RAFCON, they need to be marked in the source code.
All you need to do for this is wrapping it in the ``gettext`` function ``_()``, e.g. ``_("Hello world")``.

Strings in glade files do not need to be marked, but the ``label`` property needs an attribute ``translatable="yes"``:

.. code:: xml

    <object class="GtkButton">
        <property name="label" translatable="yes">Translatable button label</property>
        ...
    </object>

Generating a translation template
"""""""""""""""""""""""""""""""""

If new translatable strings have been added or you want to create translations for a new language, a POT file
(Portable Object Template) has to be created:

.. code:: bash

    cd /dir/to/rafcon/repository
    ./bin/i18n-gen-msg

This will create a `rafcon.pot` file in the `source/rafcon/locale` folder.

Updating an existing translation
""""""""""""""""""""""""""""""""

If you want to update a translation, open the according PO file, located in `source/rafcon/locale` (e.g. `de.po`)
with Poedit. Then go to `Catalog` => `Update from POT file` and select the generated `rafcon.pot` file. This step
is optional, if no new translatable strings have been added.

With Poedit, you can add new translations for strings comfortably. Alternatively, you can directly edit the PO files
with the text editor of your choice.

Creating a new translation
""""""""""""""""""""""""""

Open Poedit. Go to `File` => `New From POT/PO File...` and enter the name of the new language.
Start adding your translations. When you are done, store the file in `source/rafcon/locale` with the language' locale
as name, e.g. `de.po` for the German translation.
