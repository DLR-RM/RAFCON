
from gtkmvc import View

class DataFlowListView(View):
    builder = './glade/DataFlowListWidget.glade'
    top = 'data_flow_list_view'


    def __init__(self):
        View.__init__(self)

    pass