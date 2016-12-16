Discussion
==========

This page is intended as a public discussion for the development of new
features in `RAFCON <home.rst>`__.

Run core on a different host than the GUI host
----------------------------------------------

Especially for mobile robots, the RAFCON core should run directly on the
robot, while the GUI is shown on a separate computer. From the GUI, it
should at least be possible to control the execution of the
state-machine. Further on, it would be helpful to not only see the
currently executed state, but also the data port values.

At the beginning, both hosts are required to have the same state-machine
loaded, which is in read-only mode. Later on, it should be possible to
edit state-machines on the GUI hosts. These editions would then be
synced to the remote host.

Communication channels
""""""""""""""""""""""

The host running the GUI is called *master*, the remote host running
only the core is called *slave*.

control execution
    master => slave
    commands: start, stop, pause, ...
    data: integers corresponding to finite list of commands

execution status
    slave => master
    list of active states with their execution status
    data: dictionary ``{state_id: execution_status[, ...]}``

handshakes
    master slave
    e.g. checksum/hash for ensuring identical state-machines, list of
    available libraries
    data: depends on handshakes, possibly strings

data port values
    slave => master
    values of output data ports (sufficient?)
    data: nested dictionary
    ``{state_id: {port_id: data[, ...]}[, ...}}``

core state-machine
    master => slave
    initial transfer of whole state-machine
    data: file-system (zipped?)

Messaging Libraries
"""""""""""""""""""

Minimum requirements:

-  Support for at least `C(++) <http://www.cplusplus.com/>`__ and `Python <https://www.python.org/>`__
-  lightweight
-  Open Source

Possible libraries that can be used for communications between the two
(or more?) hosts.

`OpenDDS <http://www.opendds.org/>`__
    From the website:
    *OpenDDS is an open-source C++ implementation of the Object
    Management Group's specification "Data Distribution Service for
    Real-time Systems". Although OpenDDS is itself developed in C++,
    Java and JMS bindings are provided so that Java applications can use
    OpenDDS.*
    *It offers the following default transport protocols (IPv4 and
    IPv6): TCP/IP, RTPS/UDP, UDP/IP, IP multicast*
    *QoS: Real-time Delivery, Bandwidth, Redundancy, Persistence*
    `Licence <http://www.opendds.org/license.html>`__: *Since OpenDDS is
    open source and free of licensing fees, you are free to use, modify,
    and distribute the source code, as long as you include this
    copyright statement.*
    `Last
    commit <https://github.com/objectcomputing/OpenDDS/commits/master>`__
    (checked on Feb 2, 2016): Jan 28, 2016
    `Limited Python support <https://github.com/forrestv/pyDDS>`__, no
    longer maintained?
    complex
    `Heavily Used <http://portals.omg.org/dds/who-is-using-dds-2/>`__,
    also by NASA => Space related
    `Interesting DDS
    News <http://www.omg.org/news/meetings/tc/dc-13/special-events/DDS_Information_Day-agenda_.htm>`__
    DDS seems also very relevant for non-realtime applications as the
    first wrapper always mentioned for the native c++ libraries is Java
    This is the best
    `discussion <http://design.ros2.org/articles/ros_on_dds.html>`__
    about DDS found in the www so far.

`ZeroMQ <http://zeromq.org/>`__
    From the website:
     *Ø Connect your code in any language, on any platform.*
     *Ø Carries messages across inproc, IPC, TCP, TIPC, multicast.*
     *Ø Smart patterns like pub-sub, push-pull, and router-dealer.*
     *Ø High-speed asynchronous I/O engines, in a tiny library.*
     *Ø Backed by a large and active open source community.*
     *Ø Supports every modern language and platform.*
     *Ø Build any architecture: centralized, distributed, small, or
    large.*
     *Ø Free software with full commercial support.*
    `Licence <http://zeromq.org/area:licensing>`__: *The libzmq library
    is licensed under the GNU Lesser General Public License V3 plus a
    static linking exception.*
    `Last commit <https://github.com/zeromq/libzmq/commits/master>`__
    (checked on Feb 2, 2016): Feb 1, 2016
    `Python bindings <http://zeromq.org/bindings:python>`__

`nanomsg <http://nanomsg.org/>`__
    From the website:
    *nanomsg is a socket library that provides several common
    communication patterns. It aims to make the networking layer fast,
    scalable, and easy to use. Implemented in C, it works on a wide
    range of operating systems with no further dependencies.*
    `Licence <http://nanomsg.org/index.html>`__: *It is licensed under
    MIT/X11 license.*
    `Last commit <https://github.com/nanomsg/nanomsg/commits/master>`__
    (checked on Feb 2, 2016): Dec 19, 2015
    `Python bindings <https://github.com/tonysimpson/nanomsg-python>`__
    `Differences between nanomsg and
    ZeroMQ <http://nanomsg.org/documentation-zeromq.html>`__
    `nanomsg vs
    ZeroMQ <http://bravenewgeek.com/a-look-at-nanomsg-and-scalability-protocols/>`__
    bad documentation, especially for the Python wrapper

`Spread <http://www.spread.org/>`__
    From the website:
    *The Spread toolkit provides a high performance messaging service
    that is resilient to faults across local and wide area networks.*
    *Spread functions as a unified message bus for distributed
    applications, and provides highly tuned application-level multicast,
    group communication, and point to point support. Spread services
    range from reliable messaging to fully ordered messages with virtual
    synchrony delivery guarantees.*
    `Licence <http://www.spread.org/license/license.html>`__: *The
    Spread Open Source License is similar, but not identical to the BSD
    license. Specifically, the license includes the requirement that all
    advertising materials mentioning software that uses Spread display a
    specific acknowledgement. [...] Spread Concepts provides additional
    licensing options and commercial support for the Spread Toolkit.*
    `Latest release <http://www.spread.org/news.html>`__ (checked on Feb
    2, 2016): May 28, 2014 (no public repository found)
    `Outdated Python
    support <https://www.savarese.com/software/libssrcspread/>`__: Only
    supports Spread 4.1, current version is 4.4

`Tornado <http://www.tornadoweb.org>`__
    From the website:
    *Tornado is a Python web framework and asynchronous networking
    library, originally developed at FriendFeed. By using non-blocking
    network I/O, Tornado can scale to tens of thousands of open
    connections, making it ideal for long polling, WebSockets, and other
    applications that require a long-lived connection to each user.*
    Licence: `Apache licence
    2.0 <http://www.apache.org/licenses/LICENSE-2.0>`__
    `Last
    commit <https://github.com/tornadoweb/tornado/commits/master>`__
    (checked on Feb 4, 2016): Jan 16, 2016
    It is a web server, not really a communication library
    Only available for Python, but uses web communication technologies:
    HTTP, WebSockets

Alternative approach using Python remote objects
""""""""""""""""""""""""""""""""""""""""""""""""

After discovering the Python module
`Pyro <https://pythonhosted.org/Pyro4/index.html>`__, we will probably
try another approach. Pyro allows the usage of objects across a network.
Calls to objects existing on another host can be done as if the object
was on the calling host.

To achieve this, all core elements (states, transitions, outcomes, state
machine manager, ...) must be given a unique name and managed by a
server.