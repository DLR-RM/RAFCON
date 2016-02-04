import gtk
from gtkmvc import View
from rafcon.utils import constants


class StateIconView(View, gtk.IconView):
    top = 'state_icon_view'
    states = ["HS", "ES", "PS", "BS"]

    def __init__(self):
        View.__init__(self)
        gtk.IconView.__init__(self)

        self.set_columns(len(self.states))
        self.set_margin(0)
        self.set_spacing(0)
        self.set_row_spacing(0)
        self.set_column_spacing(0)

        liststore = gtk.ListStore(str)
        self.set_model(liststore)
        self.set_markup_column(0)

        for state in self.states:
            liststore.append(['<span font_desc="%s %s">&#x%s; ' % (constants.ICON_FONT, constants.FONT_SIZE_NORMAL,
                                                                   constants.BUTTON_ADD) + state + '</span>'])

        self['state_icon_view'] = self
