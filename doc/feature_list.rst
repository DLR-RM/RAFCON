List of Features
================

In the following, all features of RAFCON are listed and categorized into either *Core*, *GUI*, *Consistency Check* or *Misc*.
All listed features have their own, dedicated unit tests.

Core Features
-------------

.. table::
    :widths: 140, 110
    :align: left

    ==============================================   =================================================
    Features                                         Notes
    ==============================================   =================================================
    State Machines                                   Formal definition
    Hierarchies                                      For easy behavior modularization
    Concurrencies                                    Barrier and preemptive paradigm
    Library States (Libraries)                       For code reusability
    Data Flows + Data Ports                          For explicit data modeling
    Default Values for Data Ports
    Scoped Variable Concept                          Private, scoped and global data
    Every State Machine can be used as Library       Without any overhead
    Execution Capabilities:                          IDE like debugging mechanism
    - Start, Stop, Pause
    - Step Mode: Into, Over, Out
    - Backward Step                                  Optionally definable in each state
    - Execution History
    - Execution History Logging
    - Explicit Preemption Handling
    - Error Propagation
    - Execution Signals and Preemptive Waits
    Remote Monitoring                                Via Monitoring-Plugin
    Dynamic Modifications Possible
    API for Programmatic State Machine Generation
    Customizable Logging Features
    ==============================================   =================================================


GUI Features
------------

.. table::
    :widths: 140, 110
    :align: left

    ===============================================    ================================================
    Features                                           Notes
    ===============================================    ================================================
    State Machine Visualizer                           Using Gaphas
    State Machine Editor
    - Creation of State Machines Elements
    - Navigation inside State Machine incl. Zoom
    MVC Concept
    State Templates                                    For fast prototyping
    Separate Visualization of Data Flows
    Rearrangeable Ports
    Waypoints for Connections
    Focusable States
    Executing States are Highlighted
    Embedded Source Code Editor
    Code Syntax Check
    Library Tree
    State Machine Tree                                 Enables easy state selection
    Modification History
    Online Execution History
    State Description                                  Name, ID, type, link to library
    Logging Console View
    Design by Professional Interface Designers         In GTK3, css will be used
    Resizeable, Detachable and Foldable Widgets
    Restoring of User Specific GUI Layout
    Library Relocation Support Mechanism
    Dialog Boxes for User Communication
    Menu for Recently Opened State Machines
    Properties Window
    Session Restore
    Shortcut Manager
    Library Content Introspection                      i.e. show-content-flag per library state
    Editor Utility Functions:
    - Copy + Paste
    - Grouping and Ungrouping of States
    - Drag and Drop of States
    - State Type Change
    - Intelligent Auto State Connector                 e.g. connect income/outcome with closest sibling
    ===============================================    ================================================


Consistency Checks
------------------

.. table::
    :widths: 140, 110
    :align: left

    ==============================================   ================================================
    Features                                         Notes
    ==============================================   ================================================
    Data Type Check
    Data Flow Validity
    Hierarchical Consistency                         E.g. no connection to grandparents
    Data Integrity Check During Runtime
    Errors in Python Scripts are Caught              And forwarded to the next states as datum
    Library Interface Consistency
    ==============================================   ================================================


Misc Feat
-------------

.. table::
    :widths: 140, 110
    :align: left

    ==============================================   ==================================================================
    Features                                         Notes
    ==============================================   ==================================================================
    State Machine Versioning                         Via git
    Human Readable File Format                       json and yaml
    Programmable in Python                           Python 3
    Middleware Independent                           Tested with: ros, links and nodes, sensornet, and DDS
    Core and GUI Separated                           Core can run on micro-controller with slim Python setup
    Extensive Documentation                          Via Restructured Text (rst) and Sphinx
    Plugin Concept                                   Several plugins available
    Backward Compatibility                           Breaking changes are clearly outlined
    No Memory Leaks                                  See test_destruct.py in tests folder
    Continuous Integration                           Buildbot / Jenkins
    Usable in Different Robotic Domains              Used in: Space, Industry, Service
    Scalability: Tested with >4000 states
    Video Tutorials                                  Youtube (only one available, more to come)
    ==============================================   ==================================================================
