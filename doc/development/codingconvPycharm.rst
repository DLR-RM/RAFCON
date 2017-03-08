RMC Python style guide → Internal
"""""""""""""""""""""""""""""""""

**General**


`PEP-8: Python Style
Guide <http://www.python.org/dev/peps/pep-0008/>`__\ 
\ `https://www.python.org/dev/peps/pep-0008/ <https://www.python.org/dev/peps/pep-0008/>`__

**Excerpts of PEP-8**

for further explanations refer to PEP-8.

-  use 4 spaces per indentation level  

-  do not use tabs, only spaces!  

-  Limit all lines to a maximum 120 of characters.  

-  Imports should usually be on separate lines. 

-  Avoid extraneous whitespace:  

   -  Immediately inside parentheses, brackets or braces.  

   -  Immediately before the open parenthesis that starts the argument
      list of a function call:  

   -  Immediately before the open parenthesis that starts an indexing
      or slicing  

   -  More than one space around an assignment (or other) operator to
      align it with another.  

-  Use spaces around arithmetic operators  

-  Don't use spaces around the '=' sign when used to indicate a
   keyword argument or a default parameter value.  

-  Compound statements (multiple statements on the same line) are
   generally discouraged.  

-  While sometimes it's okay to put an if/for/while with a small body
   on the same line, never do this for multi-clause statements. Also
   avoid folding such long lines!  

-  comments  

   -  Comments that contradict the code are worse than no comments.
      Always make a priority of keeping the comments up-to-date when the
      code changes!  

   -  Comments should be complete sentences.  

   -  If a comment is short, the period at the end can be omitted.  

   -  Each line of a block comment starts with a # and a single space
       

   -  Use inline comments sparingly.  

-  docstrings (see
   `PEP-0257 <http://www.python.org/dev/peps/pep-0257>`__)  

   -  Write docstrings for all public modules, functions, classes, and
      methods.  

...
 

http://www.python.org/dev/peps/pep-0263/


DLR Python style guide
""""""""""""""""""""""

https://rmintra01.robotic.dlr.de/rmwiki/images/6/61/Code\_conventions\_python\_flyer.pdf

Extern:

https://ci.sc.dlr.de/jenkins/job/Python-Project-Setup-Guide/lastReleaseBuild/Python\_Project\_Setup\_Guide/

 
Naming Conventions
""""""""""""""""""

This naming conventions we wanna follow.
 
**ClassName**

- Nouns

 
**method\_name**

- starts with verb

 
**function\_name**

- starts with verb

 
**member\_var**

- Nouns

 
**\_protected**


**\_\_private**


**name\_conflict\_**


**\_\_builtIn\_\_**

 

PyCharm Coding Style/Conventions
""""""""""""""""""""""""""""""""

This description follows the GUI layout of Pycharm 5.0.5.
The coding style or conventions can be found in the menu bar at File -> Settings.
Those settings will be highlighted in the editor and help you code.
In our project we set the properties as follows to create a code which is uniform and easy to read.

CTRL + Shift + L will reformat your code. By default on CTRL + ALT + L .
Don't use it as you usually use CTRL-S to modify code like you were a batch script.

# image of showing editor -> Code Style -> (General/Python)

Editor -> Code Style
++++++++++++++++++++

The editor coding style can be found in the settings at Editor -> Code Style -> Python with the following topics.

 
**Python -> Tab and Indent**

# picture with of Tab and Indent

.. code:: Python

    Use tab character:      No
    Tab size:               4
    Indent:                 4
    Continuation Indent:    8


**Python -> Spaces**

# picture with of Spaces

Before Parentheses

.. code:: Python

    Method declaration parentheses: No
    Method call parenthesis:        No
    Left bracket:                   No


Around Operators

.. code:: Python

    Equality operators:             Yes
    Relation:                       Yes
    Bitwise:                        Yes
    Additive:                       Yes
    Multiplicative:                 Yes
    Shift:                          Yes
    Around = in named parameter     No
    Around = in keyword argument    No

 

Within

.. code:: Python

    Brackets:                  No
    Method call parentheses:   No
    Method declaration par.:   No
    Braces:                    No


Other

.. code:: Python

    Before comma:       No
    After comma:        Yes
    Before semicolon:   No
    Before „:“ :        No
    After „:“ :         Yes
    Before \\ :         Yes
    Before # :          Yes
    After # :           Yes
 

**Python -> Wrapping and Braces**

# picture with of Wrapping and Braces

.. code:: Python

    Right margin colomns:       120
    Wrap when reaching margin:  No

Keep when reformatting

.. code:: Python

    Line breaks:   Yes
    Ensure right margin: No
 

Method declaration params

.. code:: Python

    align when multiline:  Yes


Method call arguments

.. code:: Python

    align when multiline:  Yes


Force new line after colon

.. code:: Python

    Single-clause statements  No
    Multiclause statements Yes


Collections and Comprehensions

.. code:: Python

    align when multiline:  Yes

 

Import statements

.. code:: Python

    align when multiline:  Yes

 

**Python -> Blank Lines**

# picture with of Blank Lines

Keep max Blank lines

.. code:: Python

    In declaration:  2
    In code:  2

Minimum Blank Lines

.. code:: Python

    After imports:    1
    Around class:     1
    Around method:    1
    Around top-level: 2


Following is nice to know (Preferences):
++++++++++++++++++++++++++++++++++++++++

- exclusion of folders in projects in „Project Structure“ possible to reduce number of responses for "Find Usage"
  requests

- standard unit-test module in

- IDE settings → intentions
 

**Inspections**

Here you can define what PyCharm should remind you to check by highlight or underline code pieces.

#############################
# picture with of Blank Lines 

-  Access to a protected member of a class

-  Access to properties  

-  Argument passed to function is equal to default parameter value  

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

-  Classic style class usage  

-  Code compatibility inspection  

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

-  Missing, empty or incorrect docstring  

-  No encoding specified for file  

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
