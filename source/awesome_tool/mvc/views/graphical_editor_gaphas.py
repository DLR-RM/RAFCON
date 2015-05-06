import gtk

from gtkmvc import View

from gaphas import GtkView
from gaphas.item import Element, Item, NW, NE,SW, SE
from gaphas.constraint import LessThanConstraint, Constraint, Projection

from awesome_tool.utils import constants


class GraphicalEditorView(View):
    top = 'main_frame'

    def __init__(self):
        """View holding the graphical editor

        The purpose of the view is only to hold the graphical editor. The class ob the actual editor with the OpenGL
        functionality is GraphicalEditor
        """
        View.__init__(self)

        self.v_box = gtk.VBox()
        self.scroller = gtk.ScrolledWindow()
        self.editor = GtkView()
        self.editor.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#272c36'))
        self.scroller.add(self.editor)
        self.v_box.pack_end(self.scroller)

        self['main_frame'] = self.v_box

    def setup_canvas(self, canvas, zoom):
        self.editor.canvas = canvas
        self.editor.zoom(zoom)
        self.editor.set_size_request(500, 500)


class KeepWithinConstraint(Constraint):
    """Ensure that the children is within its parent

    Attributes:
     - parent_nw: NW coordinates of parent
     - parent_se: SE coordinates of parent
     - child_nw: NW coordinates of child
     - child_se: SE coordinates of child
    """

    def __init__(self, parent_nw, parent_se, child_nw, child_se, margin=None):
        super(KeepWithinConstraint, self).__init__(parent_nw[0], parent_nw[1], parent_se[0], parent_se[1],
                                                   child_nw[0], child_nw[1], child_se[0], child_se[1])
        self.parent_nw = parent_nw
        self.parent_se = parent_se
        self.child_nw = child_nw
        self.child_se = child_se

        self.margin = margin
        if margin is None or margin < (parent_se[0].value - parent_nw[0].value) / 1000.:
            self.margin = (parent_se[0].value - parent_nw[0].value) / 1000.
            print self.margin

    def solve_for(self, var=None):
        """
        Ensure that the children is within its parent
        """
        # Left edge (west)
        if self.parent_nw[0].value > self.child_nw[0].value - self.margin:
            print "left too far west"
            self.child_nw[0].value = self.parent_nw[0].value + self.margin
        # Right edge (east)
        if self.parent_se[0].value < self.child_se[0].value + self.margin:
            print "too far east"
            self.child_se[0].value = self.parent_se[0].value - self.margin
        # Upper edge (north)
        if self.parent_nw[1].value > self.child_nw[1].value - self.margin:
            print "too far north"
            self.child_nw[1].value = self.parent_nw[1].value + self.margin
        # Lower edge (south)
        if self.parent_se[1].value < self.child_se[1].value + self.margin:
            print "too far south"
            self.child_se[1].value = self.parent_se[1].value - self.margin


class StateView(Element):
    """ A State has 4 handles (for a start):
     NW +---+ NE
     SW +---+ SE
    """

    def __init__(self, size):
        super(StateView, self).__init__(size[0], size[1])
        self.min_width = 0.0001
        self.min_height = 0.0001
        self.width = size[0]
        self.height = size[1]

    def setup_canvas(self):

        canvas = self.canvas
        parent = canvas.get_parent(self)
        if parent is not None:
            assert isinstance(parent, StateView)
            solver = canvas.solver
            self_nw_abs = canvas.project(self, self.nw_pos())
            self_se_abs = canvas.project(self, self.se_pos())
            parent_nw_abs = canvas.project(parent, parent.nw_pos())
            parent_se_abs = canvas.project(parent, parent.se_pos())
            less_than = KeepWithinConstraint(parent_nw_abs, parent_se_abs, self_nw_abs, self_se_abs)
            self.c = solver.add_constraint(less_than)

        # Registers local constraints
        super(StateView, self).setup_canvas()

    def draw(self, context):
        c = context.cairo
        c.set_line_width(0.5)
        nw = self._handles[NW].pos
        c.rectangle(nw.x, nw.y, self.width, self.height)
        if context.hovered:
            c.set_source_rgba(.8, .8, 1, .8)
        else:
            c.set_source_rgba(1, 1, 1, .8)
        c.fill_preserve()
        c.set_source_rgb(0, 0, 0.8)
        c.stroke()

    def nw_pos(self):
        return self._handles[NW].pos

    def se_pos(self):
        return self._handles[SE].pos

    def pre_update(self, context):
        super(StateView, self).pre_update(context)
        # parent = self.canvas.get_parent(self)
        # if parent is None:
        #     return
        # for v in self.c.variables():
        #     self.c.mark_dirty(v)
        # self.canvas.solver.request_resolve_constraint(self.c)

    def post_update(self, context):
        super(StateView, self).post_update(context)
        # parent = self.canvas.get_parent(self)
        # if parent is None:
        #     return
        # print "post", self.parent_nw_abs[0].value, "<", self.self_nw_abs[0].value
        # print self.canvas.solver.constraints



# class GraphicalEditor(GtkView):
#
#     pass