#!/usr/bin/env python3
# Example 1: execution_log_viewer.py your_execution_log.shelve xxxxxxx.run_id.00000000000000000003
# Example 2: rafcon_execution_log_viewer your_execution_log.shelve xxxxxxx.run_id.00000000000000000003
from rafcon.gui.views.utils.single_widget_window import SingleWidgetWindowView
from rafcon.gui.views.execution_log_viewer import ExecutionLogTreeView
from rafcon.gui.controllers.utils.single_widget_window import SingleWidgetWindowController
from rafcon.gui.controllers.execution_log_viewer import ExecutionLogTreeController

from rafcon.utils import log
logger = log.get_logger("rafcon.gui.execution_log_viewer")

def main():
    from gi.repository import Gtk  # import here to avoid warning
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("file", help="path to the log file")
    parser.add_argument("run_id", help="optional run_id of history item to select", default=None, nargs='?')
    parser.add_argument("title_addition", help="optional string added to window title", default='', nargs='?')
    args = parser.parse_args()

    # extract state machine name for display in window title
    if "rafcon_execution_log_" in args.file and ".shelve" in args.file:
        state_machine_name = args.file.split("rafcon_execution_log_")[-1].split(".shelve")[0]
    else:
        logger.error(f"Wrong execution log file format: {args.file}")
        return

    # single widget window generation with respective size and title
    single_view = SingleWidgetWindowView(ExecutionLogTreeView, 1024, 786, f"Execution Log Viewer: '{state_machine_name}' {args.title_addition}")
    log_tree_ctrl = SingleWidgetWindowController(None, single_view, ExecutionLogTreeController, args.file, args.run_id)

    Gtk.main()


if __name__ == "__main__":
    main()