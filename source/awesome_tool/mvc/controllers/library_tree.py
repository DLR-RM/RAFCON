import gtk
import gobject

import statemachine.singleton
from utils import log
logger = log.get_logger(__name__)


#TODO: comment

class LibraryTreeController():  # (Controller):

    # actually no model needed for non modifiable library tree => take data from the library_manager
    # if libraries can be added by the GUI in the future a model will be needed
    def __init__(self, model=None, view=None):
        #Controller.__init__(self, model, view)
        self.library_tree_store = gtk.TreeStore(str, gobject.TYPE_PYOBJECT)
        view.set_model(self.library_tree_store)
        self.view = view
        self.view.connect('cursor-changed', self.on_cursor_changed)
        self.update()

    def register_adapters(self):
        pass

    def update(self):
        print "Update of library_tree controller called"
        self.library_tree_store.clear()
        for library_key, library_item in statemachine.singleton.library_manager.libraries.iteritems():
            #print library_key
            #print library
            self.insert_rec(None, library_key, library_item)

    def insert_rec(self, parent, library_key, library_item):
        #logger.debug("Add new library to tree store: %s" % library_key)
        tree_item = self.library_tree_store.insert_before(parent, None, (library_key, library_item))
        if isinstance(library_item, dict):
            #logger.debug("Found library container: %s" % library_key)
            for child_key, child_item in library_item.iteritems():
                self.insert_rec(tree_item, child_key, child_item)

    def on_cursor_changed(self, widget):
        (model, row) = self.view.get_selection().get_selected()
        # path = model.get_path(row)
        library_key = model[row][0]
        library = model[row][1]
        #print "Model and selected row: %s, %s" % (model, row)
        print "The library state should be inserted into the statemachine"


if __name__ == '__main__':
    from mvc.views import LibraryTreeView, SingleWidgetWindowView
    from mvc.controllers import SingleWidgetWindowController

    import mvc.main as main

    main.setup_path()
    main.check_requirements()
    [ctr_model, logger, ctr_state, gvm_model, emm_model] = main.create_models()

    v = SingleWidgetWindowView(LibraryTreeView, width=50, height=100)
    c = SingleWidgetWindowController(ctr_model, v, LibraryTreeController)

    gtk.main()