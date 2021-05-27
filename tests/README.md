## Unit-Tests
This directory contains all code for testing the software with pytest

Simply run the test suite with "py.test" in this or the parent directory. The -v option gives you more details, e.g. also showing passed tests.

### IMPORTANT note

Do not import any rafcon specific modules at the top of a test file!

* if pytest collects the tests it will import those modules
* by importing them, pytest will already create objects
* these created objects will then create problems in multi-threaded gtkmvc Observers
* as for those Observers it is relevant which thread created which objects!
