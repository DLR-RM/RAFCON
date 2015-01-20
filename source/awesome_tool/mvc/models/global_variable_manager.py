from gtkmvc import ModelMT
from utils.vividict import Vividict
import statemachine.singleton
import gobject
from gtk import ListStore
import gtk


class GlobalVariableManagerModel(ModelMT):

    global_variable_manager = statemachine.singleton.global_variable_manager

    __observables__ = ("global_variable_manager",)

    def __init__(self, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        self.global_variables_list_store = ListStore(gobject.TYPE_PYOBJECT)
        self.update_global_variables_list_store()

    def update_global_variables_list_store(self):
        tmp = ListStore(gobject.TYPE_PYOBJECT)
        for key, value in self.global_variable_manager.global_variable_dictionary.iteritems():
            #print "global_key: ", key
            #print "global value: ", value
            tmp.append([[key, value]])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, self.compare_global_variables)
        tms.sort_column_changed()
        tmp = tms
        self.global_variables_list_store.clear()
        for elem in tmp:
            self.global_variables_list_store.append(elem)

    def compare_global_variables(self, treemodel, iter1, iter2, user_data=None):
        path1 = treemodel.get_path(iter1)[0]
        path2 = treemodel.get_path(iter2)[0]
        # get key of first variable
        name1 = treemodel[path1][0][0]
        #print "name1: ", name1
        # get key of second variable
        name2 = treemodel[path2][0][0]
        #print "name2: ", name2
        name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
        name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
        if name1_as_bits == name2_as_bits:
            return 0
        elif name1_as_bits > name2_as_bits:
            return 1
        else:
            return -1
