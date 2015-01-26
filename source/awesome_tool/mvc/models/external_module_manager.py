from gtkmvc import ModelMT
import gobject
from gtk import ListStore, TreeStore
import gtk
import inspect
import copy

from utils.vividict import Vividict
import statemachine.singleton
from mvc.models.external_module import ExternalModuleModel

#TODO: comment

class ExternalModuleManagerModel(ModelMT):

    external_module_manager = statemachine.singleton.external_module_manager
    external_modules = []

    __observables__ = ("external_module_manager", "external_modules")

    def __init__(self, meta=None):
        """Constructor
        """

        ModelMT.__init__(self)  # pass columns as separate parameters

        if isinstance(meta, Vividict):
            self.meta = meta
        else:
            self.meta = Vividict()

        self.external_modules = []
        self.reset_external_module_model()

        self.external_modules_tree_store = TreeStore(str, str, str)
        self.update_external_modules_list_store()
        self.reset_external_module_model()

    def update_external_modules_list_store_short(self):
        self.external_modules_tree_store.clear()
        for key, external_module in self.external_module_manager.external_modules.iteritems():
            tree_iter = self.external_modules_tree_store.append(None, [external_module.name, external_module.status, " "])
            for member in inspect.getmembers(external_module.external_module_class, predicate=inspect.ismethod):
                self.external_modules_tree_store.append(tree_iter, [str(member[0]), " ", "insert"])

    def reset_external_module_model(self):
        self.external_modules = []
        for key, em in self.external_module_manager.external_modules.iteritems():
            #print em
            self.external_modules.append(ExternalModuleModel(em, self))

    def update_external_modules_list_store(self):
        #print "update_external_modules_list_store"
        tmp = TreeStore(str, str, str)
        for key, external_module in self.external_module_manager.external_modules.iteritems():
            #print "external module key: ", key
            #print "external module value: ", external_module
            #print "external module status: ", external_module.status
            tree_iter = tmp.append(None, [external_module.name, external_module.status, " "])
            for member in inspect.getmembers(external_module.external_module_class, predicate=inspect.ismethod):
                #print member[0]
                tmp.append(tree_iter, [str(member[0]), " ", "insert"])
        tms = gtk.TreeModelSort(tmp)
        tms.set_sort_column_id(0, gtk.SORT_ASCENDING)
        tms.set_sort_func(0, self.compare_external_modules)
        tms.sort_column_changed()
        tmp = tms
        self.external_modules_tree_store.clear()
        for elem in tmp:
            #print "element of tmp: ", elem
            #parent_row_iter = self.external_modules_tree_store.append(None, [copy.copy(elem[0]), copy.copy(elem[1]),
            #                                                          copy.copy(elem[2])])
            parent_row_iter = self.external_modules_tree_store.append(None, elem)
            child_row_iter = elem.iterchildren()
            #print child_row_iter
            try:
                while True:
                    row_element = child_row_iter.next()
                    #print row_element
                    self.external_modules_tree_store.append(parent_row_iter, row_element)
            except StopIteration:
                #print "last iter element found"
                pass

    def compare_external_modules(self, treemodel, iter1, iter2):
        path1 = treemodel.get_path(iter1)[0]
        #print path1
        path2 = treemodel.get_path(iter2)[0]
        #print path2
        # get key of first variable
        name1 = treemodel[path1][0]
        #print "name1: ", name1
        # get key of second variable
        name2 = treemodel[path2][0]
        #print "name2: ", name2
        name1_as_bits = ' '.join(format(ord(x), 'b') for x in name1)
        name2_as_bits = ' '.join(format(ord(x), 'b') for x in name2)
        if name1_as_bits == name2_as_bits:
            return 0
        elif name1_as_bits > name2_as_bits:
            return 1
        else:
            return -1

