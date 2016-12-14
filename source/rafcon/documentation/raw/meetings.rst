
Here all protocols and agendas for the `RAFCON <home.rst>`__ (developer)
meetings** are collected. The meeting takes place every second week on
Monday, 14:30 in meeting room 2326. If you have
questions/problems/requests regarding RAFCON, you are welcome to join.

2017-01-09
==========

-  discuss: `How to save state machines. Or: is
   \_paths\_to\_remove\_before\_sm\_save
   required? <https://rmc-github.robotic.dlr.de/common/rafcon/issues/235>`__

2016-12-14
==========

attendees: `Sebastian <user:brun_sb>`__, `Michael <user:vilz_mi>`__,
`Franz <user:stei_fn>`__

-  Sebastian:

   -  major API change (``statemachine => core``, ``mvc => gui``): Mind
      your plugins!
   -  new unit test to verify backward compatibility
   -  next minor release will drop much backward compatibility code
   -  fixed some issues found while working with RAFCON intensively

-  Michael:

   -  Wrote `wiki entry <plugins.rst#dds-monitoring-plugin>`__ about
      his DDS plugin
   -  Published further plugin:
      `ar\_rafcon\_plugin <https://rmc-github.robotic.dlr.de/flyrob/ar_rafcon_plugin>`__

-  Franz:

   -  Rewrote test to run test thread as main thread
   -  all gui tests should be adapted: gtk main loop in extra thread,
      test thread is main thread
   -  pyc files are now generated and released
   -  install directory changed from ``lib/python2.7/rafcon`` to
      ``source/rafcon``

-  discuss license issue, see `issue
   #218 <https://rmc-github.robotic.dlr.de/common/rafcon/issues/218>`__
   and the `separate
   repository <https://rmc-github.robotic.dlr.de/beck-lelk/rafcon-meta>`__

   -  open source licence should not prohibit commercial usage, but
      copy-left should force user to make their software open source,
      too (which makes commercial usages hard)
   -  we should discuss this with e.g. Freek in a later meeting

-  Discussed open issues
-  Jekyll currently not working with GitHub, but probably required for
   homepage

   -  could be installed locally
   -  force Stefan Dombrowski to make it working by visiting him
      frequently

2016-11-28
==========

attendees: beld\_rc, stei\_fn, beck\_lk, vilz\_mi, brun\_sb

-  martin l.: Anfrage wegen Erfahrungsaustausch: Kurz brainstormen
   "Kollaboratives Arbeiten"... vielleicht nach dem Meeting
   `RMC-Semminar <https://rmintra01.robotic.dlr.de/wiki/RMC-Seminar>`__: stei\_fn and brun\_sb care about
-  discuss about `action
   handling <https://rmc-github.robotic.dlr.de/common/rafcon/issues/243>`__
   and what that means: low priority issue
-  discuss about the culture on github and that some people don't like
   to read/write that much and prefer to talk about things. This may
   help to become faster/more concrete: do as you want, think before you
   write
-  discuss: `Rename/Move execution classes and
   enums <https://rmc-github.robotic.dlr.de/common/rafcon/issues/238>`__:
   assigned to brun\_sb
-  discuss location of resources, install paths in `RMPM <https://rmintra01.robotic.dlr.de/wiki/Rmpm>`__:
   stei\_fn moves all resource files in one folder name e.g. "share" and
   changes the pt file
-  clipboard: cut directly or still wait until paste action (similar to
   excel): cut directly
-  Should we clean up after the tests have been executed ... the build
   pc tmp folder could get full: delete RAFCON files on shutdown of the
   start.py, dedicated hook for the unit tests, except the auto backup
   folder

2016-11-14
==========

agenda:

-  external editor
-  selection: Strg for inverse selection and Shift for rubber band
-  print unit test: evtl. replace by github hooks
-  license issue
