
.. figure:: assets/RAFCON_Logo_Farbe_RGB.png
   :alt: Official RAFCON logo
   :width: 90 %
   :align: center


RAFCON
======

RAFCON (**R**\ MC **a**\ wesome [or advanced] **F**\ low **Con**\ trol, new name, aka Awesome tool) is an effort of
DLR RMC to create a new flow control tool, based on existing ones. On the wiki page `Ablaufsteuerung
<https://rmintra01.robotic.dlr.de/wiki/Ablaufsteuerung>`__, you can follow the previous discussions that led to this
tool. Summing up, the target of this software is to combine the advantages and features of flow control tools
like `Bubbles <https://rmintra01.robotic.dlr.de/wiki/Bubbles>`__ or `SMACH <http://wiki.ros.org/smach>`__
without inheriting their drawbacks.


.. figure:: assets/RAFCON_GUI_screenshot.png
   :width: 90 %
   :alt: Screenshot of RAFCON with an example state machine
   :align: center

RAFCON is designed and coded from scratch. The architecture was developed completely independent of Bubbles and with
modern design patterns and coding conventions in mind. The design is very flexible, e. g. allowing the core flow
control part from being executed independently from any GUI.

RAFCON is actively being used, for example for the `SBC <SBC>`__, for
`SMErobotics <https://rmintra01.robotic.dlr.de/wiki/SMErobotics>`__ or in the context of `HCR <https://rmintra01.robotic.dlr.de/wiki/HCR>`__. You
can use it as a kind of mission control center, for autonomous mobile
robots, for skill programming and many more things.


.. toctree::
   :maxdepth: 1
   :numbered:
   :caption: Table of Contents

   concepts.rst
   getting_started.rst
   changelog.rst
   tutorials.rst
   configuration.rst
   autobackup.rst
   gui_guide.rst
   plugins.rst
   faq.rst
   development/development.rst
   api/rafcon.rst



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

