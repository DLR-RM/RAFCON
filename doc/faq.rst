FAQ
===

On this page, we collect Frequently Ask Questions (FAQ) about :ref:`RAFCON`. At the moment, there are three
categories of questions concerning `Core`_, `API`_ or `GUI`_. Some more answered questions can be found in our
`list of closed issues labeled "question" <https://github.com/DLR-RM/RAFCON/issues?q=label%3Aquestion+is%3Aclosed>`__.
If you have a new question, please `create an issue <https://github .com/DLR-RM/RAFCON/issues/new>`__.

Core
----

Questions that concern the core functionality and state machine execution.

.. _faq_core_config:

Where I can configure the RAFCON core?
""""""""""""""""""""""""""""""""""""""

The core RAFCON configuration file is per default situated here in
``~/.config/rafcon/config.yaml``, but can also be passed to RAFCON using the
command line parameter ``-c /path/to/your/core/config.yaml``. For further explanation see :ref:`Configuration`.

.. _faq_initialization_global_classes:

Where can instances of global objects be hold?
""""""""""""""""""""""""""""""""""""""""""""""

Global objects can be initialized at any point using the global variable manager. Storing such variables in the
global variable manager (most of the time) only makes sense, when storing them per reference. Keep in mind:
Global variables are ugly and make your state machine less modular. Use them as little as possible!

.. _faq_concurrency:

How does concurrency work?
""""""""""""""""""""""""""

A concurrency state executes all child states concurrently. A barrier
concurrency state waits for all its children to finish their execution.
A preemptive concurrency state only waits for the first state to finsih and preempts the
others. Each state gets its own thread when it is called, so they run
completely independently. Also each state gets its own memory space,
i.e. all input data of the a state is copied at first and then passed to
the state.

Direct interaction between two concurrent branches inside a
concurrency state is not possible in RAFCON, except by using global
variables (please use them with care, as heavily using them make your state machine less modular).
The reasoning for this is that synchronization of many processes is a hot topic and very error prone.
Our recommendation is:

1. Distribute your data to parallel branches
2. Do stuff in parallel
3. After concurrent branches finished: Collect data of branches and process it
4. Distribute data it again
5. Do stuff in parallel etc.

Monitoring of different system states works via preemption and by using global variables as e.g. global signals.
A separate resource manager and world model to hold the information about the system is definitely needed for more
complex use cases. RAFCON is not a tool for holding complex data, so don't try to use it for this.

.. (please always keep in mind the difference between the state of a task and
    the state of a part a system; RAFCON was only created to manager the former)

See also :ref:`How does preemption work? How do I implement preemptable states correctly? <faq_preemption>`

.. _faq_execution_control:

How does execution control – including stepping mode – work?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

In the execution widget below the graphical editor, the execution of a
state machine can be controlled.

.. figure:: _static/Rafcon_execution_buttons.png
   :alt: Screenshot of RAFCON with an example state machine
   :width: 90 %
   :align: center

Here the user can start, pause and stop the state machine. Furthermore,
a step mode can be activated.

.. figure:: _static/Rafcon_execution_buttons_broad.png
   :alt: Screenshot of RAFCON with an example state machine
   :width: 90 %
   :align: center

In the step mode, the useer can trigger four kinds of step: "Step
Into", "Step Over", "Step Out" and "Backward Step".

The "Step Into" simply executes the next state in the state machine. So
the execution goes down and up the hierarchy.

The "Step Over" makes a step on the same hierarchy level, independent on
how many sub-states the next state will trigger. If the execution reaches
the end of the hierarchy it steps out to the hierarchy.

The "Step Out" executes all states in the current hierarchy until the
execution reaches an outcome of the current hierarchy.

The "Backward Step" triggers a backward step with respect to the current execution history.
Before and after the execution of each state all the context data (i.e. the scoped data) of the current hierarchy is
stored. The scoped data includes all the date that was
given to the current container state as input and that was created by
the child states with their outputs. A backward step now loads all the
scoped data which was valid after the execution of the state, executes
the state in backward mode and then loads the scoped data which was
valid before executing the state. Executing a state in backward mode
means executing an optional
``def backward_execute(self, inputs, outputs, gvm)`` function. The
inputs and outputs of the function are the input data of the state
(defined by its data flows) loaded from the current scoped data. If the
``backward_execute`` function is not defined, nothing is executed at
all. For an example backward-stepping state machine, have a look at the
"functionality\_examples" in the RAFCON Git repository:
``[path_to_git_repo]/share/examples/functionality_examples``.

.. _faq_pause_stop:

What does pause and stop do?
""""""""""""""""""""""""""""

Pausing a state machine prevents the current state to "take" the next
transition. Furthermore a paused-event is triggered for each state.

Stopping a state state machine also prevents the current state to "take"
the next transition. Instead of taking the transition selected by the stopped
state, the execution runs the state connected to the "preempted" outcome
of the stopped state. If no state is connected to the "preempted" outcome, the
current state hierarchy is left with the "preempted" outcome. Stopping a
state does not stop the thread of the state itself. It only triggers a
preempted-event for each state.

For information on how to correctly listen to pause or preempted events
inside a state, see :ref:`What happens if the state machine is paused? How can I pause running services, e. g. the robot? <faq_pause>`.

.. _faq_step_within_state:

Stepping through a state machine is cool, but I need to step through the code within a state. Can I do this?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

RAFCON has no special feature for this, but Python has. It is called `the Python Debugger
<https://docs.python.org/3/library/pdb.html>`__. All you have to do is inserting the following line of code into your
``execute()`` method:

.. code:: python

    import pdb; pdb.set_trace()

Alternatively, you can also use `the IPython Debugger ipdb <https://github.com/gotcha/ipdb>`__, giving you e.g. syntax
highlighting and better tracebacks:

.. code:: python

    import ipdb; ipdb.set_trace()

If this line of code is hit, the execution pauses and you need to switch to the terminal where you started RAFCON. You
can now, for example, inspect the parameters of the ``execute()`` method by entering ``a``. This will output something
like

.. code::

    inputs = {'counter': 3}
    outputs = {}
    gvm = <rafcon.core.global_variable_manager.GlobalVariableManager object at 0x7ff6a6541090>

A good tutorial about the powerful possibilities you have within the debugger can be found on `Real Python
<https://realpython.com/python-debugging-pdb/>`__.

.. _faq_preemption:

How does preemption work? How do I implement preemptable states correctly?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Preemption is achieved in *preemptive concurrency states*. All direct
children of these states are executed in parallel in separate threads.
These direct children can be of all kinds of states: execution states,
libraries or any type of container. The direct child, which finishes its execution first
(by returning an outcome), causes all sibling states to stop (preempt).
If all siblings have been preempted, the execution of the preemptive
concurrency state is finished.

When a state is preempted, the preemption starts at the innermost
running child state and propagates up: First, the preempted flag of the
innermost running children is set to True. Then it is waited until the
state returns an outcome. The outcome itself is ignored, as a preempted
state is always left on the preempted outcome. If the preempted outcome
is connected, the connected state is executed. Otherwise, the hierarchy
is left and the parent state is preempted in the same way, until the
preemptive concurrency state is reached.

States have the possibility to define an action to be executed when
being preempted. This is intended e. g. for closing any open resources.
For this, the user connects a state with the desired logic to the
preempted outcome of the state opening the resource or to its parent.
For direct children of a preemptive concurrency state, no preemption
routine can be defined. In this case another hierarchy state has to be
introduced.

**Running states are only requested to preempt but are not and cannot be
forced to preempt.** This means that states should run as short as
possible. If this is not feasible, the user has to ensure that a state
is preemptable. If a state contains a loop, the user should check in
each iteration, whether the flag ``self.preempted`` is True and stop in
this case. If a state needs to pause, ``self.preemptive_wait(time)`` or
``self.wait_for_interruption()`` (see next question) should be used
instead of ``time.sleep(time)``. The former method is preempted if the
state is urged to preempt, the latter is not. It returns True if the
wait time wasn't reached, i. e. if the method was preempted before. If
None (or nothing) is passed to ``self.preemptive_wait(time)``, the
method waits infinitely for being preempted. Note that preemption is not
only caused by sibling states within a preemptive concurrency state, but
states are also preempted if the execution of the whole state machine is
stopped (by the user clicking "Stop").

This should also be kept in mind when developing libraries. As a user
could use libraries in Preemptive Concurrency States, libraries should
be designed in this way.

.. _faq_pause:

What happens if a state machine is paused? How can I pause running services, e. g. the robot?
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The basic behavior is simple: If a state machine is paused, no more
transition is being followed. I. e., if a state returns an outcome, the
execution is stopped at this outcome. When the execution is resumed (by
clicking the "Run" button), the execution continues at this outcome.

Yet, states are not forced to pause, just as for preemption. Only the
flag ``self.paused`` is set. Therefore, states should be implemented
with care, if they run for a longer time. For this, one can use two
helper methods, ``self.wait_for_interruption(timeout=None)`` and
``self.wait_for_unpause(timeout=None)``. Alternatively, one can directly
access the Python ``threading.Event``\ s ``self._started``,
``self._paused``, ``self._preempted``, ``self._interrupted`` and ,
``self._unpaused``. The "interrupted" event is a combination of "paused"
and "stopped"; "unpaused" is a combination of "started" and "stopped".
An example implementation can be seen in the following:

.. code:: python

    def execute(self, inputs, outputs, gvm):
        self.logger.info("Starting heartbeat")

        for _ in xrange(10):
            self.logger.info("pulse")
            self.wait_for_interruption(1)

            if self.preempted:
                return "preempted"
            if self.paused:
                self.logger.debug("Heart paused")
                self.wait_for_unpause()
                if self.preempted:
                    return "preempted"
                self.logger.debug("Heart reanimated")
        return 0


An execution state with this code snippet would print "pulse" once per
second (``self.wait_for_interruption(1)``). The wait command is
interrupted, if either the user clicks "pause" or the state is preempted
(state machine is stopped or a child state running in parallel in a preemptive concurrency state finishes).
Therefore, the two event types are checked. If the state is to be
preempted, the state follows that request
(``if self.preempted: return "preempted"``). If the execution was
paused, the state waits for a resume (``self.wait_for_unpause()``). The
wait command is interrupted either by the continuation of the execution
or by a complete stop of the execution. The former manifests in the
``self.started`` flag to be set, the latter by the set of the
``self.preempted`` flag.

If an external service is involved, e. g. for commanding a robot, that
service might also be paused. For this, one can pass the respective services to the robot.
This requires the external service to be written in Python.

.. _faq_state_abortion:

How to handle a state abortion correctly?
"""""""""""""""""""""""""""""""""""""""""

As arbitrary python code is allowed in a state, the execution of a state
can raise arbitrary python errors. If an error is raised the state if
left via the "aborted" outcome. Furthermore the error of the state is
stored and passed to the next state as an input port with the name
"error". The error (e.g. its type) can be checked and used for error
handling mechanisms. If no state is connected to the "aborted" outcome
of the aborted state, the error is propagated upwards in the hierarchy
until a state is handling the abortion or the state machine is left. An
example state machine on how to use such a error handling can look like
is given in
``$RAFCON_GIT_REPO_PATH/tests/assets/unit_test_state_machines/error_propagation_test``.
If the error handling state is a hierarchy state the "error" input data
port must be manually forwarded to the first child state i.e. a
input\_data port for the hierarchy and the child state has to created
and connected.

.. _faq_jsonconversion:

How does python-jsonconversion handle string types?
"""""""""""""""""""""""""""""""""""""""""""""""""""

Serialized strings are stored in a file in ASCII encoding, but they are
read from a file as unicode. Thus explicit conversions to ASCII has to
done if the type of the string matters.


.. _faq_event_based:

Why is RAFCON not event-based?
""""""""""""""""""""""""""""""

**tl; dr**: RAFCON was created to enable the development of goal-driven behavior, in contrast to reactive behavior
(for which many frameworks rely on events).
However, RAFCON can be used to implement reactive behaviors as well. Quite elegantly, using observers!

Long Answer:
RAFCON state machines are no state machines in the classical sense.
A state in RAFCON is not a state like in a FSM (i.e., a system state), but in which a certain action is executed.
Thus, a state is rather related to flowchart block. The HPFD formalization underlying RAFCON is defined in (see https://elib.dlr.de/112067/).
Related to robotics, a RAFCON state is a state in the robot's behavior (e.g., performing pick-up action)
and not a robot's system state (battery at 20V, position at [x, y]).

We have employed RAFCON for many goal-driven scenarios (see https://dlr-rm.github.io/RAFCON/projects).
In goal-driven scenarios (in contrast to reactive ones) all changes of the environment are an effect of the robot's own actions.
There are only a few "external events" the robot has to react to (low voltage, bad signal etc.).

As stated before, RAFCON can not only be used to build goal-driven behavior but also to build reactive systems.
Classical state machines or statecharts can be used to implement reactive behavior as well and they DO rely on events.
Behavior trees are also a very powerful concept for reactive behavior and they do NOT rely on events.
For RAFCON, we don't rely on events either.
We employ observers to take care of events.
To implement them, you have to use (preemptive) concurrency states.
E.g. you can set up a preemptive concurrency state with 5 observers. The first one who fires decides the course of action.
You can place different observers on different hierarchy levels.
Thereby, you can define the scope of your 'event' and your 'event handler'.

We have a lot of experience with event-based state machines like boost statechart, due to our experience in the RoboCup competition
(see https://link.springer.com/chapter/10.1007/978-3-642-39250-4_5).
One lesson learned was that applying event-based state machines does not scale.

To understand this, imagine a classical use-case scenario:

* You are using a non-graphical library for programming event-based state machines like boost statechart.
* You have a hierarchical state machine with several hundreds of states inside several hierarchy layers.
* Your have 30 types of events that are continuously firing onto your state machine.
* In certain states you react to some events.
* Most of the time you neglect 90% of the events and are only interested in some events defined by the current set of active states. Nevertheless, the events arbiter has to handle *every* event, which can be inefficient.

Now try to answer the following questions, that would be raised during runtime:

1. What is the set of currently active states?
2. What events do I currently listen to?
3. What is the context (hierarchy level, possible predecessors, possible successors) of each of those state?
4. Since when do I listen to event_x? Until which state will I listen to event_x?
5. I receive an event that I cannot process know. I defer it to a later point in time (*event deferral*).

  * How long are events valid (*event caching*)?
  * Another event with the same type arrives meanwhile. Should I keep the old event (*event expiration*)?
  * Another event, which is more important arrives. Should I react to the new event and preempt the current event handling (*event priorization*)?

It is obviously not trivial! Of course, you could answer those questions, but it is cubersome and you quickly loose the overview.

Thus, we implemented a graphical user interface, where you can easily answer those questions:

1. You clearly see all currently executed states, highlighted in green in the graphical 2.5D layout and in your execution history tree!
2. Event observers are states. For active state, see answer 1.
3. You clearly see the context of each state visually drawn (if you want to have a closer look, simply zoom in)
4. Events map to observers, observers to states; their entry and exit is clearly visualized => see 3.
5. As we do use observers instead of events we don't have to care about all this complex topics using the limited power of events
   (Events are error-prone anyways. Adobe reported that 50% of all bugs are related to event handling: https://stlab.cc/legacy/figures/Boostcon_possible_future.pdf)
   Instead of complex event handling, you can create powerful observer structures also in a nested manner using hierarchies and outcome forwarding.

Concerning how to deal with the four mentioned event-challenges using observers:

* Event deferral: Use a proper hierarchical layout for your observers.
  The observer in a higher hierarchy layer will defer its events, until the execution of its child observers is finished.
* Event caching: As you have one observer per event, caching is done automatically until the observer is preempted.
* Event expiration: By preempting an observer sibling you clear the "cache" for this event. Make sure you have a proper hierarchical observer layout:
  i.e., if an observer must not clear the cache for a certain event, pull it's observer one hierarchy up!
* Event priorization: Make sure you have a proper hierarchical observer layout.
  Observers on a higher hierarchy layer can preempt observers on a lower hierarhcy level, but not vice versa.

These statements only hold true in the case of programming complex reactive systems,
in which only a subset of events is of interest in certain semantic situations.
For GUIs, in which you are normally prepared to react to the majority of events the whole time, classical event handling is of course a reasonable way to go!

Take away message: Observers are more powerful than events. RAFCON goes with observers!


.. _faq_filesystem_names:

API
---

Questions that concern the core programming interface.

.. _faq_api_examples:

Are there examples how to use the API?
""""""""""""""""""""""""""""""""""""""

Some examples can be found in the folder
``$RMPM_RAFCON_ROOT_PATH/share/examples/api`` or if you use our git-repo
see ``$RAFCON_GIT_REPO_PATH/share/examples/api``. Many more examples of
how to create a state machine using the python API can be found in
``$RAFCON_GIT_REPO_PATH/source/test/common``.

GUI
---

Questions that concern the graphical user interface.

.. _faq_gui_configuration:

Where can I configure the RAFCON GUI?
"""""""""""""""""""""""""""""""""""""

You can either use File => Settings or manually edit
``~/.config/rafcon/gui_config.yaml``. This location can also be specified
by the parameter ``-g`` in the command line. For further explanation see
:ref:`Configuration`.


.. _faq_collaboration:

How to effectively collaborate on creating state machines?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The following guidelines will help you to collaboratively work on bigger state machines:

* Use git for versioning your state machines!
* Do not work at the same time on the same state machine as your colleague! You really have to know what your are doing if you merge state machine json files!
* Clearly distribute your state machine in several modules, and create library states for these modules. Then, clearly define the interfaces of these libraries. Finally, your libraries can be developed in parallel.
* If you nevertheless encounter a git conflict either throw away the smaller part of the changes, which are conflicting, and re-create them on a healthy git version. Or try to merge (recommended only for Pros!)


.. _faq_library_interface_change:

How to handle a library interface change of a library used in a (bigger) state machine ?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Even if you have a robust and clever modularization of your code, these kind of situations will occur! There are several cases.

**The location of a library changed, but the library kept the same**:
    No problem, RAFCON will help you to relocate your libraries. Don't forget to save the library after replacing the old libraries with the new ones.

**The interface of a library changed:**
    If you just added data ports or outcomes, you are fine! RAFCON can handle these cases easily. If you removed outcomes or data ports of a library your state machine, which includes this library can become invalid. Either data flows try to connect to no more existing data ports or transitions to no more existing outcomes. You won't be able to open the invalid state machine with the default setting. In this case use the LIBRARY_RECOVERY_MODE set to True (see Core :ref:`Configuration`). This will allow you to open invalid state machines. Currently, it simply removes all connections in a hierarchy if one port in the hierarchy is missing. (Removing only the erroneous connection would of course be much more convenient, and there is already an issue for that. Feel free to contribute :-) !)

**The location and the interface of a library changed:**
    Try to avoid this case! Otherwise it will mean a good portion of work for you! The library relocation feature won't help you, as it cannot handle interface changes yet. Basically you have to cancel the library relocation process. This means that you will end up with hierarchies without connections. All the modified library states are replaced by "Hello world" dummy states. Basically, this means that you have to rebuild all hierarchies that held a link to a library, whose location and interface changed.


.. _faq_change_hierarchy:

How can the hierarchy level of a state be changed in the graphical editor after it was created?
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Moving a state into another state currently only works using cut and
paste. As the copied state won't change its size, it is preferable to
fit the sizes of the state to move and/or the target state. Then select
the state to be moved and press Ctrl+X or use the menu Edit => Cut. The
state is now in the clipboard, but is still shown. Now select the state
into which you want to move your copied state. Make sure the target
state is of type Hierarchy or Concurrency. With Ctrl+V or Edit => Paste,
the original state is moved into the target state.

If you only want to combine several states you can use the group feature. This creates a new
HierarchyState and moves the currently selected states into the new state. To use the group feature select all
states to be grouped (they have to be on one hierarchical level) and then use the group-shortcut
(STRG-G per default) or the menu bar Edit->Group entry.


The RAFCON GUI looks weird. Strange symbols are scattered all over the GUI. What can I do?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Probably RAFCON cannot find its fonts. If you installed RAFCON via pip, uninstall it and install it again.
If you checked out RAFCON's git repo, reinstall the fonts. See the :ref:`Getting Started <install_fonts>` page for
that.


Known Issues
""""""""""""

.. _faq_maximization_issue:

A window can not be un-maximized what I can do?
+++++++++++++++++++++++++++++++++++++++++++++++

Generally, this ia a problem related to your window manager
and can be caused by different screens sizes when using several monitors or similar nasty configurations.
The fastest way to solve this problem is to delete your runtime_config.yaml file which
is commonly situated at ``~/.config/rafcon/runtime_config.yaml`` and which will be generated
automatically and cleanly after removal.


.. _faq_start_issue:

Why does lauchning RAFCON sometimes blocks for several seconds?
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

This again is problem of some window managers and is related to the automatically generated config file ``~/.gtkrc``.
Simply remove this file and RAFCON should start normally again.
