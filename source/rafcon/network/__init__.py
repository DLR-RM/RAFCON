"""
.. module:: network
   :synopsis: Websocket-based remote interface between a headless RAFCON core and one or more GUI clients

"""

# Set to True in a GUI process that is attached to a remote core. Editing state machines is not
# synchronized back to the core yet, so GUI code may consult this flag to disable editing affordances.
remote_session = False
