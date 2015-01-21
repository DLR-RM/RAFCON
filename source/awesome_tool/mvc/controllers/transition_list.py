
from utils import log
logger = log.get_logger(__name__)

from gtkmvc import Controller
from gtk import ListStore


class TransitionListController(Controller):
    """Controller handling the view of transitions of the ContainerStateModel

    This :class:`gtkmvc.Controller` class is the interface between the GTK widget view
    :class:`mvc.views.transitions.TransitionListView` and the transitions of the
    :class:`mvc.models.state.ContainerStateModel`. Changes made in
    the GUI are written back to the model and vice versa.

    :param mvc.models.ContainerStateModel model: The container state model containing the data
    :param mvc.views.TransitionListView view: The GTK view showing the transitions as a table
    """

    def __init__(self, model, view):
        """Constructor
        """
        Controller.__init__(self, model, view)

    def register_view(self, view):
        """Called when the View was registered
        """

        def cell_text(column, cell_renderer, model, iter, container_model):
            col = column.get_name()
            states_store = ListStore(str, str)
            states_store.append([container_model.state.state_id, container_model.state.name])
            transition = model.get_value(iter, 0)
            for state_model in container_model.states.itervalues():
                states_store.append([state_model.state.state_id, state_model.state.name])
            if col == 'from_state_col':
                text = container_model.state.states[transition.from_state].name
                cell_renderer.set_property('text', text)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'to_state_col':
                text = 'Sel. state' if transition.to_state is None else  \
                    container_model.state.states[transition.to_state].name
                cell_renderer.set_property('text', text)
                cell_renderer.set_property('text-column', 1)
                cell_renderer.set_property('model', states_store)
            elif col == 'from_outcome_col':
                from_state = container_model.state.states[transition.from_state]
                cell_renderer.set_property('text', from_state.outcomes[transition.from_outcome].name)
            elif col == 'to_outcome_col':
                if transition.to_outcome is None:
                    cell_renderer.set_property('text', '')
                else:
                    cell_renderer.set_property('text', container_model.state.outcomes[transition.to_outcome].name)
            else:
                logger.error("Unknown column '{col:s}' in TransitionListView".format(col=col))


        view.get_top_widget().set_model(self.model.transition_list_store)

        view['from_state_col'].set_cell_data_func(view['from_state_combo'], cell_text, self.model)
        view['to_state_col'].set_cell_data_func(view['to_state_combo'], cell_text, self.model)
        view['from_outcome_col'].set_cell_data_func(view['from_outcome_combo'], cell_text, self.model)
        view['to_outcome_col'].set_cell_data_func(view['to_outcome_combo'], cell_text, self.model)


        view['from_state_combo'].connect("edited", self.on_combo_changed)
        view['to_state_combo'].connect("edited", self.on_combo_changed)

    def register_adapters(self):
        """Adapters should be registered in this method call
        """

    def on_combo_changed(self, widget, path, text):
        logger.debug("Widget: {widget:s} - Path: {path:s} - Text: {text:s}".format(widget=widget, path=path, text=text))


import gtk
import gobject
import traceback
import sys
#===========================================================
#                   Mask Edit View
#===========================================================
class MaskEditView(gtk.VBox):

    def __init__(self, mask, libmode=False):
        gtk.VBox.__init__(self)
        self.tree = MaskEditTree(mask, libmode)

        self.add_button = gtk.Button('Add')
        self.add_button.connect("clicked", self.tree.on_add)

        self.remove_button = gtk.Button('Remove')
        self.remove_button.connect("clicked", self.tree.on_remove)

        self.adv_button = gtk.Button('Advanced view')
        self.adv_button.connect("clicked", self.tree.on_adv)

        self.Hbox = gtk.HButtonBox()
        self.Hbox.add(self.add_button)
        self.Hbox.add(self.remove_button)
        self.Hbox.add(self.adv_button)

        self.pack_start(self.Hbox, expand=False, fill=True)

        scrollable = gtk.ScrolledWindow()
        scrollable.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        scrollable.add(self.tree)

        self.add(scrollable)
        self.show_all()

#===========================================================
#                   Mask Edit View
#===========================================================
class MaskEditTree(gtk.TreeView):
    '''
    @brief Used for the tab where the user can edit the library mask of a bubble.
    '''

    #===============================================================
    def __init__(self, mask, libmode=False):
        '''
        @brief constructor
        @param mask mask of the bubble that is edited
        @param libmode Library or not.
            - if it is a library, every field are editable
            - otherwise, just the value is editable
        '''
        gtk.TreeView.__init__(self)
        self.libmode = libmode
        #self.type_list = ['num', 'list', 'text']
        self.type_list = ['text'] # this is only temporarily to make it more stable
        self.typecombo = gtk.ListStore(gobject.TYPE_STRING)
        self.mask = mask

        self.set_search_column(0)
        self.set_reorderable(True)


        for x in self.type_list :
            self.typecombo.append([x])

        self.treestore = gtk.TreeStore(str, str, str, str, str)
        self.set_model(self.treestore)

        # Variable name
        self.name_cell = gtk.CellRendererText()
        self.name_cell.set_property("width-chars", 15)
        self.name_cell.set_property("editable", True)
        self.name_cell.connect('edited', self.on_name_modification)
        name_col = gtk.TreeViewColumn('Name', self.name_cell, text=0, background=3)
        self.append_column(name_col)

        # Variable Type
        self.type_cell = gtk.CellRendererCombo()
        self.type_cell.set_property("model", self.typecombo)
        self.type_cell.set_property("text-column", 0)
        self.type_cell.set_property("width", 60)
        self.type_cell.connect('edited', self.on_type_modification)
        type_col = gtk.TreeViewColumn('Type', self.type_cell, text=1, background=3)
        self.append_column(type_col)

        # Variable Value
        self.val_cell = gtk.CellRendererText()
        self.val_cell.set_property("editable", True)
        self.val_cell.connect('edited', self.on_val_modification)
        self.val_col = gtk.TreeViewColumn('Value', self.val_cell, text=2, background=4)
        self.append_column(self.val_col)

        self.set_libmode(libmode)
        self.connect("drag_data_received", self.drag_data_received_data)

        self.update()

    def drag_data_received_data(self, treeview, context, x, y, selection, info, etime):
        self.update_path()

    def update_path(self, top=None) :
        if top == None :
            next = self.treestore.get_iter_first()
        else :
            next = self.treestore.iter_children(top)
        while next :
            name, size = self.treestore.get(next, 0, 1)
            path = self.treestore.get_path(next)
            for item in self.mask.tree :
                if item[0] == name and item[2] == (size == None) :
                    self.mask.tree.tree.remove(item)
                    self.mask.tree.tree.append((name, path, (size == None)))

            self.update_path(next)
            next = self.treestore.iter_next(next)

    def set_libmode(self, bool) :
        '''
        @brief Change libmode and set the field editable accordingly
        '''
        self.libmode = bool
        self.type_cell.set_property("editable", bool)
        if bool :
            self.val_col.set_property("title", "Default Value")
        else :
            self.val_col.set_property("title", "Value")
        return

    def on_add(self, widget) :
        name = 'Unknown 0'
        i = 0
        while name in [x[0] for x in self.mask.tree.cat(True)] :
            i += 1
            name = name[:-1] + str(i)
        self.mask.tree.add_to_tree(name, True)
        self.treestore.append(None, [name, None, None, '#f0E5C7', '#f0E5c7'])
        self.update_path()

    def on_remove(self, widget) :
        self.update_path()
        tree, path = self.get_selection().get_selected_rows()
        tree, iter = self.get_selection().get_selected()
        for p in path :
            for item in self.mask.tree :
                if p == item[1] :
                    #parent = self.treestore.get_iter(p)
                    #gp = self.treestore.iter_parent(parent)
                    #children = self.treestore.iter_n_children(parent)
                    #for i in range(children) :
                        #child = self.treestore.iter_nth_child(parent,i)
                        #row = []
                        #for col in range(self.treestore.get_n_columns()) :
                            #row.append(self.treestore.get(child,col)[0])
                        #self.treestore.insert_before(gp,parent,row)
                        #self.treestore.remove(child)
                    #self.treestore.remove(parent)

                    if item[2] == False :
                        continue
                    else :
                        self.mask.tree.rem_from_tree(item[0], item[2])
        #self.mask.tree.tree_sort()
        self.update(update_path=False)

    def on_adv(self, widget) :
        tree, path = self.get_selection().get_selected_rows()
        tree, iter = self.get_selection().get_selected()
        # for p in path :
        #     for item in self.mask.tree :
        #         if p == item[1] and item[2] == False :
        #             it = self.mask.get_var(item[0])
        #             alt = MaskAdvancedView(it, self.libmode)
        #             self.mask.set_var(item[0], alt.entry)
        # self.update()

    def on_name_modification(self, cell, path, text):
        title = self.treestore[path][0]
        for i, item in enumerate(self.mask.tree) :
            if item[0] == title and item[2] == True :
                self.mask.tree[i] = (text, item[1], item[2])
                self.treestore[path][0] = text
                self.treestore.set_value(self.treestore.get_iter(path), 0, text)
                break

    #===============================================================
    def update(self, update_path=True):
        '''
        @brief Fill the mask treeview
        the last two fields are for the cell background.
        the 5th for the backgd of name, size and type
        the 6th for value
        '''
        if update_path :
            self.update_path()
        self.treestore.clear()
        dic = {}
        self.mask.tree.tree_sort()
        for item in self.mask.tree :
            if len(item[1]) == 1 :
                parent = None
            else :
                parent = dic[item[1][:-1]]

            if item[2] == True :
                dic[item[1]] = self.treestore.append(parent, [item[0], None, None, '#f0E5C7', '#f0E5c7'])
            else :
                for vals in self.mask.item :
                    if vals.name == item[0] :
                        val = vals
                if self.libmode or val.val == None :
                    dic[item[1]] = self.treestore.append(parent, [val.name, val.type, str(val.defval), 'white', '#99FF99'])
                else :
                    dic[item[1]] = self.treestore.append(parent, [val.name, val.type, str(val.val), 'white', 'white'])

    def convert_path(self, p_str) :
        res = (int(p_str[0]),)
        for i in p_str[1:] :
            if i != ':' :
                res += (int(i),)
        return res


    #===============================================================
    def on_type_modification(self, cell, path, text) :
        path = self.convert_path(path)
        self.update_path()
        item = self.mask.get_var_i(path)
        item.type = text
        if text == 'list' :
            item.defval = [0]
        else :
            item.defval = 0
        item.val = None

        self.treestore.set_value(self.treestore.get_iter(path), 1, text)

    #===============================================================
    def on_val_modification(self, cell, path, text):
        logger.debug('on_val_modification()')

        model = self.get_model()
        name = model.get_value(model.get_iter(path), 0)

        path = self.convert_path(path)
        self.update_path()
        error = []

        val = 0

        def mod_tree_val(text, item, error) :
            if self.libmode or text == None :
                if error :
                    self.treestore.set_value(self.treestore.get_iter(path), 2, str(item.defval))
                    self.treestore.set_value(self.treestore.get_iter(path), 4, 'red')
                else :
                    self.treestore.set_value(self.treestore.get_iter(path), 2, text)
                    self.treestore.set_value(self.treestore.get_iter(path), 4, '#99FF99')
            else :
                if error :
                    self.treestore.set_value(self.treestore.get_iter(path), 2, str(item.val))
                    self.treestore.set_value(self.treestore.get_iter(path), 4, 'red')
                else :
                    self.treestore.set_value(self.treestore.get_iter(path), 2, text)
                    self.treestore.set_value(self.treestore.get_iter(path), 4, 'white')


        #When no text is entered, set the value to None,
        #In this case, the default value is used instead.
        if text == '' :
            item = self.mask.get_var(name)
            if item:
                item.set_val(None)

        else :
            item = self.mask.get_var(name)
            if not item :
                print "Element %r Not Found in %r" % (path, self.mask.tree.tree)
                return
            if item.type == 'num' :
                try :
                    val = eval(text)
                    if type(val) in [int, long, float, complex] :
                        if item.test_val(val) :
                            item.set_val(val, self.libmode)
                            mod_tree_val(text, item, False)
                        else :
                            print "Error : invalid value"
                            mod_tree_val(text, item, True)
                    else :
                        print "Error : invalid type"
                        mod_tree_val(text, item, True)
                except :
                    print "Error in the given value", traceback.format_exc()#sys.exc_info()[0]
                    mod_tree_val(text, item, True)


            elif item.type == 'list' :
                try :
                    val = eval(text)
                    if type(val) in [list, tuple] :
                        if not item.test_size(len(val)) :
                            print "Error, the list entered is not of the given size"
                            error.append(path)
                            mod_tree_val(text, item, True)
                        else :
                            item.set_val(val, self.libmode)
                            mod_tree_val(text, item, False)
                    else :
                        print "Error : invalid type"
                        error.append(path)
                        mod_tree_val(text, item, True)
                except :
                    print "Error in the given value", sys.exc_info()[0]
                    error.append(path)
                    mod_tree_val(text, item, True)

            else :
                item.set_val(text, self.libmode)
                mod_tree_val(text, item, False)

        #self.update()