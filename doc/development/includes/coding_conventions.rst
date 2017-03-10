RMC Python style guide → Internal
"""""""""""""""""""""""""""""""""

**General**


`PEP-8: Python Style Guide <http://www.python.org/dev/peps/pep-0008/>`__

**Excerpts of PEP-8**

For further explanations refer to PEP-8. In this section, we highlight parts

.. raw:: html

    <div style="color:#008000;">in green that are important to us </div> and
    <div style="color:#ff0000;">in red that we don't follow that strictly.</div>

- .. raw:: html

    <div style="color:#008000;">
        use 4 spaces per indentation level
    </div>

- .. raw:: html

    <div style="color:#008000;">
        do not use tabs, only spaces!
    </div>

- .. raw:: html

    <div style="color:#ff0000;">
        Limit all lines to a maximum 120 of characters.
    </div>

- Imports should usually be on separate lines.

- Avoid extraneous whitespace:

    - .. raw:: html

        <div style="color:#008000;">
            Immediately inside parentheses, brackets or braces.
        </div>
    - .. raw:: html

        <div style="color:#008000;">
            Immediately before the open parenthesis that starts the argument list of a function call
        </div>
    - .. raw:: html

        <div style="color:#008000;">
            Immediately before the open parenthesis that starts an indexing or slicing
        </div>
    - .. raw:: html

        <div style="color:#008000;">
            More than one space around an assignment (or other) operator to align it with another.
        </div>


- .. raw:: html

    <div style="color:#008000;">
        Use spaces around arithmetic operators
    </div>

- .. raw:: html

    <div style="color:#008000;">
        Don't use spaces around the '=' sign when used to indicate a keyword argument or a default parameter value.
    </div>

- .. raw:: html

    <div style="color:#008000;">
        Compound statements (multiple statements on the same line) are generally discouraged.
    </div>

- .. raw:: html

    <div style="color:#008000;">
        While sometimes it's okay to put an if/for/while with a small body on
        the same line, never do this for multi-clause statements. Also avoid
        folding such long lines!
    </div>

- Comments

    - .. raw:: html

        <div style="color:#00ff00;">
            Comments that contradict the code are worse than no comments.
            Always make a priority of keeping the comments up-to-date when the
            code changes!
        </div>
    - Comments should be complete sentences.

    - If a comment is short, the period at the end can be omitted.

    - .. raw:: html

        <div style="color:#008000;">
            Each line of a block comment starts with a # and a single space
        </div>
    - .. raw:: html

        <div style="color:#008000;">
            Use inline comments sparingly.
        </div>

- Docstrings (see `PEP-0257 <http://www.python.org/dev/peps/pep-0257>`__)

    - .. raw:: html

        <div style="color:#008000;">
            Write docstrings for all public modules, functions, classes, and
            methods.
        </div>

...



DLR Python style guide
""""""""""""""""""""""


`DLR internal Style Guide <https://rmintra01.robotic.dlr.de/wiki/File:Code_conventions_python_flyer.pdf>`__



Naming Conventions
""""""""""""""""""

This naming conventions we wanna follow.
 
ClassName
    Nouns

method\_name
    starts with verb


function\_name
    starts with verb

member\_var
    Nouns

\_protected
    should not be accessed from outside

\_\_private
    enforces protection

name\_conflict\_
    e.g. useful for collisions with reserved names (`class_`)

\_\_builtIn\_\_
    like `__init__`

 

PyCharm Coding Style/Conventions
""""""""""""""""""""""""""""""""

This description follows the GUI layout of Pycharm 5.0.5.
The coding style or conventions can be found in the menu bar at File -> Settings.
Those settings will be highlighted in the editor and help you code.
In our project we set the properties as follows to create a code which is uniform and easy to read.

CTRL + Shift + L will reformat your code. By default on CTRL + ALT + L .
Don't use it as you usually use CTRL-S to modify code like you were a batch script.


Editor -> Code Style
++++++++++++++++++++

The editor coding style can be found in the settings at Editor -> Code Style -> Python with the following topics.

 
**Python -> Tab and Indent**

+----------------------+-----+----------------------------------------------------------+
| **Tab and Indent**   |     |                                                          |
+----------------------+-----+                                                          |
| Use tab character:   |  No |.. figure:: ../assets/pycharm_settings_tab_and_indent.png |
+----------------------+-----+   :width: 100%                                           |
| Tab size:            |  4  |   :align: center                                         |
+----------------------+-----+                                                          |
| Indent:              |  4  |                                                          |
+----------------------+-----+                                                          |
| Continuation Indent: |  8  |                                                          |
+----------------------+-----+----------------------------------------------------------+


**Python -> Spaces**

+---------------------------------+-----+----------------------------------------------------------+
| **Before Parentheses**          |     |.. figure:: ../assets/pycharm_settings_spaces.png         |
+---------------------------------+-----+   :width: 100%                                           |
| Method declaration parentheses: | No  |   :align: center                                         |
+---------------------------------+-----+                                                          |
| Method call parenthesis:        | No  |                                                          |
+---------------------------------+-----+                                                          |
| Left bracket:                   | No  |                                                          |
+---------------------------------+-----+                                                          |
| **Around Operators**            |     |                                                          |
+---------------------------------+-----+                                                          |
| Equality operators:             | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Relation operators:             | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Bitwise operators:              | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Additive operators:             | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Multiplicative operators:       | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Shift operators (<<, >>, >>>):  | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Around = in named params:       | No  |                                                          |
+---------------------------------+-----+                                                          |
| Around = in keyword args:       | No  |                                                          |
+---------------------------------+-----+                                                          |
| **Within**                      |     |                                                          |
+---------------------------------+-----+                                                          |
| Brackets:                       | No  |                                                          |
+---------------------------------+-----+                                                          |
| Method call parentheses:        | No  |                                                          |
+---------------------------------+-----+                                                          |
| Method declaration par.:        | No  |                                                          |
+---------------------------------+-----+                                                          |
| Braces:                         | No  |                                                          |
+---------------------------------+-----+                                                          |
| **Other**                       |     |                                                          |
+---------------------------------+-----+                                                          |
| Before comma:                   | No  |                                                          |
+---------------------------------+-----+                                                          |
| After comma:                    | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Before semicolon:               | No  |                                                          |
+---------------------------------+-----+                                                          |
| Before „:“ :                    | No  |                                                          |
+---------------------------------+-----+                                                          |
| After „:“ :                     | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Before \\ :                     | Yes |                                                          |
+---------------------------------+-----+                                                          |
| Before # :                      | Yes |                                                          |
+---------------------------------+-----+                                                          |
| After # :                       | Yes |                                                          |
+---------------------------------+-----+----------------------------------------------------------+


**Python -> Wrapping and Braces**

+------------------------------------+-----+-----------------------------------------------------------+
| **Right margin colomns**           | 120 |.. figure:: ../assets/pycharm_settings_wrapping_braces.png |
+------------------------------------+-----+   :width: 100%                                            |
| **Wrap when reaching margin**      | No  |   :align: center                                          |
+------------------------------------+-----+                                                           |
| **Keep when reformatting**         |     |                                                           |
+------------------------------------+-----+                                                           |
| Line breaks:                       | Yes |                                                           |
+------------------------------------+-----+                                                           |
| Ensure right margin:               | No  |                                                           |
+------------------------------------+-----+                                                           |
| **Method declaration params**      |     |                                                           |
+------------------------------------+-----+                                                           |
| align when multiline:              | Yes |                                                           |
+------------------------------------+-----+                                                           |
| **Method call arguments**          |     |                                                           |
+------------------------------------+-----+                                                           |
| align when multiline:              | Yes |                                                           |
+------------------------------------+-----+                                                           |
| **Force new line after colon**     |     |                                                           |
+------------------------------------+-----+                                                           |
| Single-clause statements:          | No  |                                                           |
+------------------------------------+-----+                                                           |
| Multi-clause statements:           | Yes |                                                           |
+------------------------------------+-----+                                                           |
| **Collections and Comprehensions** |     |                                                           |
+------------------------------------+-----+                                                           |
| align when multi-line:             | Yes |                                                           |
+------------------------------------+-----+                                                           |
| **Import statements**              |     |                                                           |
+------------------------------------+-----+                                                           |
| align when multi-line:             | Yes |                                                           |
+------------------------------------+-----+                                                           |
| **Dictionary literals**            |     |                                                           |
+------------------------------------+-----+                                                           |
| New line after '{':                | No  |                                                           |
+------------------------------------+-----+                                                           |
| Place '}' on new line:             | No  |                                                           |
+------------------------------------+-----+-----------------------------------------------------------+


**Python -> Blank Lines**

+---------------------------------+-----+-----------------------------------------------------------+
| **Keep max Blank lines**        |     |.. figure:: ../assets/pycharm_settings_blank_lines.png     |
+---------------------------------+-----+   :width: 100%                                            |
| In declaration:                 |  2  |   :align: center                                          |
+---------------------------------+-----+                                                           |
| In code:                        |  2  |                                                           |
+---------------------------------+-----+                                                           |
| **Minimum Blank Lines**         |     |                                                           |
+---------------------------------+-----+                                                           |
| After imports:                  |  1  |                                                           |
+---------------------------------+-----+                                                           |
| Around class:                   |  1  |                                                           |
+---------------------------------+-----+                                                           |
| Around method:                  |  1  |                                                           |
+---------------------------------+-----+                                                           |
| Around top-level:               |  2  |                                                           |
+---------------------------------+-----+-----------------------------------------------------------+
 

Inspections
+++++++++++

Here you can define what PyCharm should remind you to check by highlight or underline code pieces. In the following
list you will find some elements highlighted

.. raw:: html

    <div style="color:#008000;"> green (for recommend to enable it) </div> and
    <div style="color:#ff0000;"> red (for recommend to disable or leave disabled). </div>

We recommend to change some of the option to push the code style to be more readable while developing new features
and using PyCharm.


.. figure:: ../assets/pycharm_code_style_inspections.png
   :width: 100%
   :align: center


**General**

-  .. raw:: html

    <div style="color:#008000;">
        Line is longer than allowed by code style
    </div>


**Python**

-  Access to a protected member of a class

-  Access to properties  

- .. raw:: html

   <div style="color:#008000;">
        Argument passed to function is equal to default parameter value -> enable it!
   </div>

-  Assigning function call that doesn't return anything  

-  Assignment can be replaced with augmented assignment  

-  Assignment to 'for' loop or 'with' statement parameter  

-  Bad except clauses order  

-  Boolean variable check can be simplified  

-  Byte literal contains characters > 255  

-  Calling a method by class using an instance of a different class  

-  Chained comparisons can be simplified  

-  Class has no \_\_init\_\_ method  

-  Class must implement all abstract methods  

-  Class specific decorator on method outside class  

-  .. raw:: html

    <div style="color:#ff0000;">
        Classic style class usage
    </div>

-  .. raw:: html

    <div style="color:#008000;">
        Code compatibility inspection
    </div>

-  Comparison with None performed with equality operators  

-  Default argument is mutable  

-  Deprecated function, class or module  

-  Dictionary contains duplicate keys  

-  Dictionary creation could be rewritten by dictionary literal  

-  Errors in string formatting operations  

-  Exception doesn't inherit from standard ''Exception'' class  

-  File contains non-ASCII character  

-  from \_\_future\_\_ import must be the first executable statement  

-  Function call can be replaced with set literal  

-  Global variable is undefined at the module level  

-  Incompatible signatures of \_\_new\_\_ and \_\_init\_\_  

-  Inconsistent indentation  

-  Incorrect call arguments  

-  \_\_init\_\_ method that returns a value  

-  Instance attribute defined outside \_\_init\_\_  

-  Invalid interpreter configured  

-  List creation could be rewritten by list literal  

-  Method may be static  

-  Method signature does not match signature of overridden method  

-  Methods having troubles with first parameter  

-  Missed call to \_\_init\_\_ of super class  

-  .. raw:: html

    <div style="color:#008000;">
        Missing, empty or incorrect docstring
    </div>

-  .. raw:: html

    <div style="color:#ff0000;">
        No encoding specified for file
    </div>

-  Old-style class contains new-style class features  

-  Package requirements  

-  PEP 8 coding style violation  

-  PEP 8 naming convention violation  

-  Problematic nesting of decorators  

-  Property definitions  

-  Raising a new style class  

-  Raising a string exception  

-  Reassignment of method's first argument  

-  Redeclared names without usage  

-  Redundant parentheses  

-  Shadowing built-ins  

-  Shadowing names from outer scopes  

-  Single quoted docstring  

-  Statement has no effect  

-  Too broad exception clauses  

-  Trailing semicolon in statement  

-  Trying to call a non-callable object  

-  Tuple assignment balance is incorrect  

-  Tuple item assignment  

-  Type checker  

-  Type in docstring doesn't match inferred type  

-  Unbound local variable  

-  Unnecessary backslash  

-  Unreachable code  

-  Unresolved references  

-  Unused local  

-  Wrong arguments to call super
