List of Features
================


Core Features
-------------

.. table::
    :widths: 150, 25, 100
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    State Machines                                      x               Own formal definition
    States                                              x
    Transitions                                         x
    Hierarchies                                         x
    Concurrencies                                       x               Barrier and Preemptive Paradigm
    Library States (libraries)                          x
    State Templates                                     x
    Data Flows                                          x
    Default Values for Data Flows                       x
    Variable Concept:                                   x               Private, Scoped and Global Data
    Default values for libraries                        x
    Runtime values for libraries                        x
    Execution capabilities:                             x
    - Start, Stop, Pause                                x
    - Step Mode: Into, Over, Out                        x
    - Backward Step                                     x
    - Execution History                                 x
    - Execution History Logging                         x
    - Explicit Preemption Handling                      x
    - Error Propagation                                 x
    - Execution Signals and Preemptive Waits            x
    - Remote Monitoring                                 x               via Monitoring Plugin
    Dynamic Modifications Possible                      x
    API for programmatic behavior generation            x
    Logging Library incl. Different Logging Levels      x
    ==============================================      ===========     ========================


GUI Features
-------------

.. table::
    :widths: 150, 25, 100
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    MVC Concept                                         x               via pygtkmvc
    OpenGL and Gaphas support                           x
    Complex Actions:                                    x
    - Copy + Paste                                      x
    - Group                                             x
    ... todo                                            x
    ==============================================      ===========     ========================


Consistency Checks
------------------

.. table::
    :widths: 150, 25, 100
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    Data Type Check                                     x
    Data Flow Validity                                  x
    Hierarchical Consistency                            x               e.g. no connection to grandparents
    Data integrity check during runtime                 x
    Errors in python scripts are caught                 x               and forwarded to the next states as datum
    Library Interface Consistency                       x
    ==============================================      ===========     ========================

Misc Features
-------------

.. table::
    :widths: 150, 25, 100
    :align: right

    ==============================================      ===========     ========================
    Features                                            Unit Tested     Notes
    ==============================================      ===========     ========================
    State Machine Versioning                            _               via git
    State Machine Deployment                            _               via git, rmpm, conan
    Human Readable File Format                          _               json and yaml
    Programmable in python                              x               python 2 (python 3 in progress)
    Middleware independent                              x               Tested with: ros, links and nodes, sensornet, and DDS
    Core and GUI separated                              x
    Documentation via Sphinx                            _               rst
    Plugin Concept                                      _               custom plugin concept
    Usable in different robotic domains                 x               Used in: Space, Industry, Service
    Scalability:                                        x
    - Examples up to 100 states                         x               Year: 2015
    - Examples up to 1000 states                        x               Year: 2017
    - Examples up to 4000 states                        x               Year: 2018
    Example States Available                            x               see GitHub
    Backward compatibility                              x               breaking changes are clearly outlined
    No memory Leaks                                     x               see test_destruct.py in tests folder
    Backward compatibility                              x               breaking changes are clearly outlined
    Conversion between GUI types                        x               supported types: OpenGL and Gaphas
    Continuous Integration                              x               buildbot / Jenkins
    Video Tutorials                                     x               Youtube (only one available, more to come)
    ==============================================      ===========     ========================