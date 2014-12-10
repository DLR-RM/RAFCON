
from gtkmvc import View
from gtk import ListStore

class TransitionListView(View):
    builder = './glade/TransitionListWidget.glade'
    top = 'transition_list_view'


    def __init__(self):
        View.__init__(self)


        # model = ListStore(str, str, str, str)
        # model.append(["y", "State A", "C", "d"])
        # model.append(["y", "State B", "C", "d"])
        # model.append(["x", "State B", "C", "d"])
        # states = ListStore(str, str)
        # states.append(["x", "State A"])
        # states.append(["y", "State B"])
        # self['transition_list_view'].set_model(model)
        #
        # self['from_state_col'].add_attribute(self['from_state_combo'], "text", 1)
        # self['from_state_combo'].set_property("text-column", 1)
        # self['from_state_combo'].set_property("model", states)
        #
        # self['to_state_combo'].set_property("text", 0)
        # self['to_state_combo'].set_property("model", states)
        # self['to_state_col'].add_attribute(self['to_state_combo'], "text", 0)
        # self['to_state_combo'].set_property("editable", True)
        # self['to_state_combo'].set_property("text-column", 0)

    pass