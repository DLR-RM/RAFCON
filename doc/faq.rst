FAQ
===

On this page, we collect Frequently Ask Questions (FAQ) about
:ref:`RAFCON`. At the moment, there are three categories of
questions concerning `Core`_, `API`_ or
`GUI`_. If you have a new question, please feel free to add
those to the section `New Questions`_.

Core
----

Questions that concern the core functionality/execution.

.. _faq_core_config:

Where I can configure the RAFCON core?
""""""""""""""""""""""""""""""""""""""

The core RAFCON configuration file is generally situated here
``~/.config/rafcon/config.yaml`` but also can be handed using a path as
the config-folder parameter ``-c`` in the command line. For further
explanation see :ref:`Configuration`.

.. _faq_initialization_global_classes:

Where can instances of global classes be initialized (e.g. a LN-client)?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Global classes can be initialized at any point. You can e. g. create a
state at the beginning of the state machine with name "Init LN-client",
which does the initialization within some Python module. If this module
is imported in another state, it can use the initialized client, as the
Python shell didn't change.

.. _faq_concurrency:

How does concurrency work?
""""""""""""""""""""""""""

A concurrency state executes all child states concurrently. A barrier
concurrency state waits for all its children to finish their execution.
A preemptive concurrency state only waits for the first and preempts the
others. Each state gets its own thread when it is called, so they run
completely independently. Also each state gets its own memory space,
i.e. all input data of the a state is copied at first and then passed to
the state. Interaction between two concurrent branches inside a
concurrency state is not possible in RAFCON, except by using global
variables (please use them with care, as heavily using them quickly
leads to a bad state machine design).

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

In the step mode, the use now can trigger four kinds of step: "Step
Into", "Step Over", "Step Out", "Backward Step".

The "Step Into" simply executes the next state in the state machine. So
the execution goes down and up the hierarchy.

The "Step Over" makes a step on the same hierarchy level, independent on
how many substates the next state will trigger. If the execution reaches
the end of the hierarchy, it steps out to the next higher hierarchy.

The "Step Out" executes all states in the current hierarchy until the
execution reaches an outcome of the current hierarchy.

The "Backward Step" triggers a backward step with respect to the current
execution history. Before and after the execution of each state, the
scoped data is stored. The scoped data includes all the data that was
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
all. For example backward-stepping state machines, have a look at the
"functionality\_examples" in the RAFCON Git repository:
``[path_to_git_repo]/share/examples/functionality_examples``.

.. _faq_pause_stop:

What does pause and stop do?
""""""""""""""""""""""""""""

Pausing a state machine prevents the current state to "take" the next
transition. Furthermore a paused-event of each state is triggered.

Stopping a state state machine also prevents the current state to "take"
the next transition. Instead of taking the transition selected by the
state, the execution runs the state connected to the "preempted" outcome
of the state. If no state is connected to the "preempted" outcome, the
current state hierarchy is left with the "preempted" outcome. Stopping a
state does not stop the thread of the state itself. It only triggers a
preempted-event of each state.

For information on how to correctly listen to pause or preempted events
inside a state, see `What happens if the state machine is paused? How can I pause running services, e. g. the robot?`_.

.. _faq_preemption:

How does preemption work? How do I implement preemptable states correctly?
""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Preemption is achieved in *preemptive concurrency states*. All direct
children of these states are executed in parallel in separate threads.
These direct children can be all kinds of states: execution states,
libraries or any container. The direct child finishing execution first
(by returning an outcome) causes all sibling states to stop (preempt).
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
be designed in this way. For further comprehension consider the state
machine example in share/examples/tutorials/simple\_preemption\_example in the
project folder.

.. _faq_pause:

What happens if the state machine is paused? How can I pause running services, e. g. the robot?
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

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
(state machine is stopped or a state running in parallel finishes).
Therefore, the two event types are checked. If the state is to be
preempted, the state follows that request
(``if self.preempted: return "preempted"``). If the execution was
paused, the state waits for a resume (``self.wait_for_unpause()``). The
wait command is interrupted either by the continuation of the execution
or by a complete stop of the execution. The former manifests in the
``self.preempted`` flag to be set, the latter by the set of the
``self.started`` flag.

If an external service is involved, e. g. for commanding a robot, that
service might also be paused. For this, one can pass the one or more
events to that service. This requires the external service to be written
in Python.

:: _faq_state_abortion:

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
``$RAFCON_GIT_REPO_PATH/tests/asserts/unit_test_state_machines/error_propagation_test``.
If the error handling state is a hierarchy state the "error" input data
port must be manually forwarded to the first child state i.e. a
input\_data port for the hierarchy and the child state has to created
and connected.

:: _faq_jsonconversion:

How does python-jsonconversion handle string types?
"""""""""""""""""""""""""""""""""""""""""""""""""""

Serialized strings are stored in a file in ASCII encoding, but they are
read from a file as unicode. Thus explicit conversions to ASCII has to
done if the type of the string matters.

:: _faq_filesystem_names:

API
---

Questions that concern the core programming interface.

:: _faq_api_examples:

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

:: _faq_gui_configuration:

Where can I configure the RAFCON GUI?
"""""""""""""""""""""""""""""""""""""

You can either use File => Settings or manually edit
``~/.config/rafcon/gui_config.yaml``. This location can also be specified
by the parameter ``-c`` in the command line. For further explanation see
:ref:`Configuration`.

:: _faq_change_hierarchy

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

If you only wanna combine several states to have them on one level or to
encapsulate them you can use the group feature. This will create a new states
HierarchyState and move the currently selected states into those. So you select all
states to group on one hierarchical level and then use the group-shortcut
typically STRG-G or the menu bar Edit->Group.

Known Issues
""""""""""""

:: _faq_maximization_issue:

A window can not be un-maximized what I can do?
+++++++++++++++++++++++++++++++++++++++++++++++

This is generally a problem the is related to your window manager of your desktop
and can be caused by different screens sizes or similar nasty configurations.
The fastest way to solve this problem is to delete your runtime_config.yaml file which
is commonly situated at ``~/.config/rafcon/runtime_config.yaml`` and which will be generated
automatically and cleanly after removal.

:: _faq_start_issue:

Why start of RAFCON GUI sometimes never finish?
+++++++++++++++++++++++++++++++++++++++++++++++

This can happens on some distributions because of a ``.gtkrc`` file in the home directory so
``~/.gtkrc``. Remove this file from your home directory and RAFCON should start correctly again.


New Questions
-------------

Please `create an issue <https://github.com/DLR-RM/RAFCON/issues/new>`__ for your question.
