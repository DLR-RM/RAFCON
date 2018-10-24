List of Features
================


Core Features
-------------

.. table::
    :widths: 140, 20, 110
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    State Machines                                      x               Formal definition
    Hierarchies                                         x               For easy behavior modularization
    Concurrencies                                       x               Barrier and preemptive paradigm
    Library States (Libraries)                          x               For code reusability
    Data Flows + Data Ports                             x               For explicit data modeling
    Default Values for Data Ports                       x
    Scoped Variable Concept                             x               Private, scoped and global data
    Default Values for Libraries                        x               Defined by the library creator
    Runtime Values for Libraries                        x               Defined by the user
    Every State Machine can be used as Library          x               Without any overhead
    Execution Capabilities:                             x               IDE like debugging mechanism
    - Start, Stop, Pause                                x
    - Step Mode: Into, Over, Out                        x
    - Backward Step                                     x               Optionally definable in each state
    - Execution History                                 x
    - Execution History Logging                         x
    - Explicit Preemption Handling                      x
    - Error Propagation                                 x
    - Execution Signals and Preemptive Waits            x
    Remote Monitoring                                   x               Via Monitoring-Plugin
    Dynamic Modifications Possible                      x
    API for Programmatic Behavior Generation            x
    Logging Library incl. Different Logging Levels      x
    ==============================================      ===========     ========================


GUI Features
-------------

.. table::
    :widths: 140, 20, 110
    :align: right

    ===============================================     ===========     ========================
    Features                                            Unit Tested     Notes
    ===============================================     ===========     ========================
    State Machine Visualizer                            x               OpenGL and Gaphas support
    State Machine Editor                                x
    - Creation of State Machines Elements               x
    - Navigation inside State Machine incl. Zoom        x
    MVC Concept                                         x
    State Templates                                     x               For fast prototyping
    Separate Visualization of Data Flows                x
    Rearrangeable Ports                                 x
    Waypoints for Connections                           x
    Focusable States                                    x
    Executing States are Highlighted                    x
    Embedded Source Code Editor                         x
    Code Syntax Check                                   x
    Library Tree                                        x
    State Machine Tree                                  x               Next to graphical editor for easy state
    selection
    Modification History                                x
    Online Execution History                            x
    State Description                                   x               Name, ID, type, link to library
    Logging Console View                                x
    Proper CSS-like Design                              x               In GTK3, css will be used
    Resizeable, Detachable and Foldable Widgets         x
    Restoring of User Specific GUI Layout               x
    Library Relocation Support Mechanism                x
    Dialog Boxes for User Communication                 x
    Menu for Recently Opened State Machines             x
    Properties Window                                   x
    Session Restore                                     x
    Shortcut Manager                                    x
    Library Content Introspection                       x               i.e. show-content-flag per library state
    Editor Utility Functions:                           x
    - Copy + Paste                                      x
    - Grouping and Ungrouping of States                 x
    - Drag and Drop of States                           x
    - State Type Change                                 x
    - Right Click Menus                                 x
    - Intelligent State Connector                       x               e.g. connect income/outcome with closest sibling
    ===============================================     ===========     ========================


Consistency Checks
------------------

.. table::
    :widths: 140, 20, 110
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    Data Type Check                                     x
    Data Flow Validity                                  x
    Hierarchical Consistency                            x               E.g. no connection to grandparents
    Data Integrity Check During Runtime                 x
    Errors in Python Scripts are Caught                 x               And forwarded to the next states as datum
    Library Interface Consistency                       x
    ==============================================      ===========     ========================

Misc Features
-------------

.. table::
    :widths: 140, 20, 110
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    State Machine Versioning                            _               Via git
    State Machine Deployment                            _               Via git, rmpm, conan
    Human Readable File Format                          _               json and yaml
    Programmable in Python                              x               Python 2 (Python 3 in progress)
    Middleware Independent                              x               Tested with: ros, links and nodes, sensornet, and DDS
    Core and GUI Separated                              x               Core can run on micro-controller with slim Python setup
    Documentation via Sphinx                            _               Via Restructured Text (rst)
    Plugin Concept                                      _               Custom plugin concept
    Backward Compatibility                              x               Breaking changes are clearly outlined
    No Memory Leaks                                     x               See test_destruct.py in tests folder
    Continuous Integration                              x               Buildbot / Jenkins
    Usable in Different Robotic Domains                 x               Used in: Space, Industry, Service
    Scalability:                                        x
    - Examples up to 700 States                         x               Year: 2015
    - Examples up to 1500 States                        x               Year: 2017
    - Examples up to 4000 States                        x               Year: 2018
    Example States Available                            x               See GitHub
    Video Tutorials                                     x               Youtube (only one available, more to come)
    ==============================================      ===========     ========================