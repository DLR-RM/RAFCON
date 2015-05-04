from gtk import Button
from awesome_tool.utils import constants

import sys, getopt


def set_button_children_size_request(widget):
        try:
            for child in widget.get_children():
                if isinstance(child, Button):
                    child.set_size_request(constants.BUTTON_MIN_WIDTH, constants.BUTTON_MIN_HEIGHT)
                else:
                    set_button_children_size_request(child)
        except AttributeError:
            return


def get_opt_paths():
        sm_path = None
        gui_path = None
        net_path = None

        try:
            opts, args = getopt.getopt(sys.argv[1:], "hs:g:n:", ["sm=", "gui=", "net="])
        except getopt.GetoptError:
            print '%s -s <path/to/statemachine_config> -g <path/to/gui_config>' % sys.executable
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                print '%s -s <path/to/statemachine_config> -g <path/to/gui_config>' % sys.executable
                sys.exit()
            elif opt in ("-s", "--sm"):
                sm_path = arg
            elif opt in ("-g", "--gui"):
                gui_path = arg
            elif opt in ("-n", "--net"):
                net_path = arg

        return sm_path, gui_path, net_path