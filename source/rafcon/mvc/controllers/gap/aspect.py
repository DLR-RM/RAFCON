from simplegeneric import generic

from gaphas.aspect import ConnectionSink, Connector


class ItemHandleInMotion(object):
    """
    Move a handle (role is applied to the handle).
    This class replaces the default one in order to adjust the snap distance in move()
    """

    def __init__(self, item, handle, view):
        self.item = item
        self.handle = handle
        self.view = view
        self.last_x, self.last_y = None, None

    def start_move(self, pos):
        self.last_x, self.last_y = pos
        canvas = self.item.canvas

        cinfo = canvas.get_connection(self.handle)
        if cinfo:
            canvas.solver.remove_constraint(cinfo.constraint)

    def move(self, pos, distance):
        item = self.item
        handle = self.handle
        view = self.view

        v2i = view.get_matrix_v2i(item)

        x, y = v2i.transform_point(*pos)

        self.handle.pos = (x, y)

        sink = self.glue(pos, distance)

        # do not request matrix update as matrix recalculation will be
        # performed due to item normalization if required
        item.request_update(matrix=False)

        return sink

    def stop_move(self):
        pass

    def glue(self, pos, distance):
        """
        Glue to an item near a specific point.

        Returns a ConnectionSink or None.
        """
        item = self.item
        handle = self.handle
        view = self.view

        if not handle.connectable:
            return None

        connectable, port, glue_pos = \
                view.get_port_at_point(pos, distance=distance, exclude=(item,))

        # check if item and found item can be connected on closest port
        if port is not None:
            assert connectable is not None

            connector = Connector(self.item, self.handle)
            sink = ConnectionSink(connectable, port)

            if connector.allow(sink):
                # transform coordinates from view space to the item space and
                # update position of item's handle
                v2i = view.get_matrix_v2i(item).transform_point
                handle.pos = v2i(*glue_pos)
                return sink
        return None

HandleInMotion = generic(ItemHandleInMotion)