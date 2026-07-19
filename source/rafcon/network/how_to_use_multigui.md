# Usage

- **start core in terminal**
`rafcon_core --server 9999 -o <path_to_statemachine>`

- **start gui in other terminal**
`rafcon --connect ws://localhost:9999`

## Web GUI

- **start core with websocket server + web server**
`rafcon_core --server 9999 --web-port 8880`

- **open in browser**
`http://localhost:8880`

State machines can be opened directly from the browser: the *Libraries* tab in the left sidebar
shows all libraries from the core's `LIBRARY_PATHS` (double-click opens one as a state machine,
like in the GTK GUI), and the path field below it opens any state machine folder on the core's
file system. Passing `-o <path_to_statemachine>` on the command line still works to preload one.

The web GUI (see `source/rafcon/web/README.md`) visualizes the state machines with live execution
highlighting and controls the execution (start/pause/stop/step modes). Like the remote GTK GUI it
is a viewer — editing is not synchronized over the network layer.
