"""
Module implements guides when moving items and handles around.
"""

from simplegeneric import generic

from gaphas.aspect import InMotion, PaintFocused, ItemInMotion, ItemPaintFocused
from gaphas.item import Item, Element, Line

from rafcon.mvc.mygaphas.aspect import HandleInMotion, ItemHandleInMotion
from rafcon.mvc.mygaphas.items.state import StateView


class ItemGuide(object):
    """
    Get edges on an item, on which we can align the items.
    """

    def __init__(self, item):
        self.item = item

    def horizontal(self):
        """
        Return horizontal edges (on y axis)
        """
        return ()

    def vertical(self):
        """
        Return vertical edges (on x axis)
        """
        return ()


Guide = generic(ItemGuide)


@Guide.when_type(Element)
class ElementGuide(ItemGuide):

    def horizontal(self):
        y = self.item.height
        return (0, y/2, y)

    def vertical(self):
        x = self.item.width
        return (0, x/2, x)


@Guide.when_type(Line)
class LineGuide(ItemGuide):
    """
    Support guides for orthogonal lines.
    """

    def horizontal(self):
        line = self.item
        if line.orthogonal:
            if line.horizontal:
                for i, h in enumerate(line.handles()):
                    if i % 2 == 1:
                        yield h.pos.y
            else:
                for i, h in enumerate(line.handles()):
                    if i % 2 == 0 and i > 0:
                        yield h.pos.y

    def vertical(self):
        line = self.item
        if line.orthogonal:
            if line.horizontal:
                for i, h in enumerate(line.handles()):
                    if i % 2 == 0 and i > 0:
                        yield h.pos.x
            else:
                for i, h in enumerate(line.handles()):
                    if i % 2 == 1:
                        yield h.pos.x


class Guides(object):

    def __init__(self, v, h):
        self.v = v
        self.h = h

    def vertical(self):
        return self.v

    def horizontal(self):
        return self.h


class GuideMixin(object):
    """
    Helper methods for guides.
    """

    MARGIN = 2

    def find_vertical_guides(self, item_vedges, pdx, height, excluded_items):
        view = self.view
        item = self.item
        i2v = self.view.get_matrix_i2v
        margin = self.MARGIN
        items = []
        for x in item_vedges:
            items.append(view.get_items_in_rectangle((x - margin, 0, margin*2, height)))
        try:
            guides = map(Guide, reduce(set.union, map(set, items)) - excluded_items)
        except TypeError:
            guides = []

        vedges = set()
        for g in guides:
            for x in g.vertical():
                vedges.add(i2v(g.item).transform_point(x, 0)[0])
        dx, edges_x = self.find_closest(item_vedges, vedges)
        return dx, edges_x


    def find_horizontal_guides(self, item_hedges, pdy, width, excluded_items):
        view = self.view
        item = self.item
        i2v = self.view.get_matrix_i2v
        margin = self.MARGIN
        items = []
        for y in item_hedges:
            items.append(view.get_items_in_rectangle((0, y - margin, width, margin*2)))
        try:
            guides = map(Guide, reduce(set.union, map(set, items)) - excluded_items)
        except TypeError:
            guides = []

        # Translate edges to canvas or view coordinates
        hedges = set()
        for g in guides:
            for y in g.horizontal():
                hedges.add(i2v(g.item).transform_point(0, y)[1])

        dy, edges_y = self.find_closest(item_hedges, hedges)
        return dy, edges_y


    def get_excluded_items(self):
        """
        Get a set of items excluded from guide calculation.
        """
        item = self.item
        view = self.view

        excluded_items = set(view.canvas.get_all_children(item))
        excluded_items.add(item)
        excluded_items.update(view.selected_items)
        return excluded_items


    def get_view_dimensions(self):
        try:
            allocation = self.view.allocation
        except AttributeError, e:
            return 0, 0
        return allocation.width, allocation.height


    def queue_draw_guides(self):
        view = self.view
        try:
            guides = view.guides
        except AttributeError:
            return

        w, h = self.get_view_dimensions()

        for x in guides.vertical():
            view.queue_draw_area(x-1, 0, x+2, h)
        for y in guides.horizontal():
            view.queue_draw_area(0, y-1, w, y+2)


    def find_closest(self, item_edges, edges):
        delta = 0
        min_d = 1000
        closest = []
        for e in edges:
            for ie in item_edges:
                d = abs(e - ie)
                if d < min_d:
                    min_d = d
                    delta = e - ie
                    closest = [e]
                elif d == min_d:
                    closest.append(e)
        if min_d <= self.MARGIN:
            return delta, closest
        else:
            return 0, ()


@InMotion.when_type(Item)
class GuidedItemInMotion(GuideMixin, ItemInMotion):
    """
    Move the item, lock position on any element that's located at the same
    location.
    """

    def get_excluded_items(self):

        root_items = self.view.canvas.get_root_items()
        exclude_items = set(self.view.canvas.get_all_children(root_items[0]))

        for child in self.view.canvas.get_all_children(root_items[0]):
            if (isinstance(child, StateView) and isinstance(self.item, StateView) and
                    self.item.hierarchy_level == child.hierarchy_level and child is not self.item):
                exclude_items.remove(child)

        return exclude_items

    def move(self, pos):
        item = self.item
        view = self.view

        transform = view.get_matrix_i2v(item).transform_point
        w, h = self.get_view_dimensions()

        px, py = pos
        pdx, pdy = px - self.last_x, py - self.last_y

        excluded_items = self.get_excluded_items()

        item_guide = Guide(item)
        item_vedges = [transform(x, 0)[0] + pdx for x in item_guide.vertical()]
        dx, edges_x = self.find_vertical_guides(item_vedges, pdx, h, excluded_items)

        item_hedges = [transform(0, y)[1] + pdy for y in item_guide.horizontal()]
        dy, edges_y = self.find_horizontal_guides(item_hedges, pdy, w, excluded_items)

        newpos = px + dx, py + dy

        # Call super class, with new position
        sink = super(GuidedItemInMotion, self).move(newpos)

        self.queue_draw_guides()

        view.guides = Guides(edges_x, edges_y)

        self.queue_draw_guides()

        return sink


    def stop_move(self):
        self.queue_draw_guides()
        try:
            del self.view.guides
        except AttributeError:
            # No problem if guides do not exist.
            pass


@HandleInMotion.when_type(Item)
class GuidedItemHandleInMotion(GuideMixin, ItemHandleInMotion):

    def move(self, pos):

        sink = super(GuidedItemHandleInMotion, self).move(pos)

        if not sink:
            item = self.item
            handle = self.handle
            view = self.view
            x, y = pos
            v2i = view.get_matrix_v2i(item)

            excluded_items = self.get_excluded_items()

            w, h = self.get_view_dimensions()

            dx, edges_x = self.find_vertical_guides((x,), 0, h, excluded_items)

            dy, edges_y = self.find_horizontal_guides((y,), 0, w, excluded_items)

            newpos = x + dx, y + dy

            x, y = v2i.transform_point(*newpos)

            self.handle.pos = (x, y)
            #super(GuidedItemHandleInMotion, self).move(newpos)

            self.queue_draw_guides()

            view.guides = Guides(edges_x, edges_y)

            self.queue_draw_guides()

            item.request_update()


    def stop_move(self):
        self.queue_draw_guides()
        try:
            del self.view.guides
        except AttributeError:
            # No problem if guides do not exist.
            pass


@PaintFocused.when_type(Item)
class GuidePainter(ItemPaintFocused):

    def paint(self, context):
        try:
            guides = self.view.guides
        except AttributeError:
            return

        cr = context.cairo
        view = self.view
        allocation = view.allocation
        w, h = allocation.width, allocation.height

        cr.save()
        try:
            cr.set_line_width(1)
            cr.set_source_rgba(0.0, 0.0, 1.0, 0.6)
            for g in guides.vertical():
                cr.move_to(g, 0)
                cr.line_to(g, h)
                cr.stroke()
            for g in guides.horizontal():
                cr.move_to(0, g)
                cr.line_to(w, g)
                cr.stroke()
        finally:
            cr.restore()

# vim:sw=4:et:ai
