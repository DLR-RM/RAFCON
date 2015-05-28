from gaphas.tool import Tool
from gaphas.aspect import HandleInMotion, Connector, HandleFinder, HandleSelection, ConnectionSink
from gaphas.item import NW
from simplegeneric import generic

from awesome_tool.mvc.views.gap.connection import ConnectionView

import gtk

from awesome_tool.mvc.views.gap.state import StateView


class MyHoverTool(Tool):

    def __init__(self, view=None):
        super(MyHoverTool, self).__init__(view)

    def on_motion_notify(self, event):
        # if self.view.hovered_item:
        #     canvas = self.view.hovered_item.canvas
        #     parent = canvas.get_parent(self.view.hovered_item)
        #     if parent is not None:
        #         assert isinstance(parent, StateView)
        #         self_nw_abs = canvas.project(self.view.hovered_item, self.view.hovered_item.handles()[NW].pos)
        #         print event.x - self.view.hovered_item.handles()[NW].pos.x
        #         print event.y - self.view.hovered_item.handles()[NW].pos.y
        #     print self.view.hovered_item.handles()[NW].pos
        # print event.x
        # print event.y
        pass


# ------------------------------------------------------------------
# -----------------------------SNAPPING-----------------------------
# ------------------------------------------------------------------

class MyItemHandleInMotion(object):
    """
    Move a handle (role is applied to the handle)
    """

    GLUE_DISTANCE = 5.0

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

    def glue(self, pos, distance=GLUE_DISTANCE):
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


MyHandleInMotion = generic(MyItemHandleInMotion)


class MyHandleTool(Tool):
    """
    Tool for moving handles around.

    By default this tool does not provide connecting handles to another item
    (see `ConnectHandleTool`).
    """

    def __init__(self, view=None):
        super(MyHandleTool, self).__init__(view)
        self.grabbed_handle = None
        self.grabbed_item = None
        self.motion_handle = None

    def grab_handle(self, item, handle):
        """
        Grab a specific handle. This can be used from the PlacementTool
        to set the state of the handle tool.
        """
        assert item is None and handle is None or handle in item.handles()
        self.grabbed_item = item
        self.grabbed_handle = handle

        selection = HandleSelection(item, handle, self.view)
        selection.select()

    def ungrab_handle(self):
        """
        Reset grabbed_handle and grabbed_item.
        """
        item = self.grabbed_item
        handle = self.grabbed_handle
        self.grabbed_handle = None
        self.grabbed_item = None
        if handle:
            selection = HandleSelection(item, handle, self.view)
            selection.unselect()

    def on_button_press(self, event):
        """
        Handle button press events. If the (mouse) button is pressed on
        top of a Handle (item.Handle), that handle is grabbed and can be
        dragged around.
        """
        view = self.view

        item, handle = HandleFinder(view.hovered_item, view).get_handle_at_point((event.x, event.y))

        if handle:
            # Deselect all items unless CTRL or SHIFT is pressed
            # or the item is already selected.

            if not (event.state & (gtk.gdk.CONTROL_MASK | gtk.gdk.SHIFT_MASK)
                    or view.hovered_item in view.selected_items):
                del view.selected_items

            view.hovered_item = item
            view.focused_item = item

            self.motion_handle = None

            self.grab_handle(item, handle)

            return True

    def on_button_release(self, event):
        """
        Release a grabbed handle.
        """
        # queue extra redraw to make sure the item is drawn properly
        grabbed_handle, grabbed_item = self.grabbed_handle, self.grabbed_item

        if self.motion_handle:
            self.motion_handle.stop_move()
            self.motion_handle = None

        self.ungrab_handle()

        if grabbed_handle:
            grabbed_item.request_update()
        return True

    def on_motion_notify(self, event):
        """
        Handle motion events. If a handle is grabbed: drag it around,
        else, if the pointer is over a handle, make the owning item the
        hovered-item.
        """
        view = self.view
        if self.grabbed_handle and event.state & gtk.gdk.BUTTON_PRESS_MASK:
            canvas = view.canvas
            item = self.grabbed_item
            handle = self.grabbed_handle
            pos = event.x, event.y

            if not self.motion_handle:
                self.motion_handle = MyHandleInMotion(item, handle, self.view)
                self.motion_handle.start_move(pos)
            if isinstance(item, ConnectionView) and item.from_handle() is handle:
                self.motion_handle.move(pos, 5.0 / ((item.hierarchy_level + 1) * 2))
            elif isinstance(item, ConnectionView) and item.to_handle() is handle:
                self.motion_handle.move(pos, 5.0 / (item.hierarchy_level * 2))
            else:
                self.motion_handle.move(pos, 5.0)

            return True


class MyConnectHandleTool(MyHandleTool):
    """
    Tool for connecting two items.

    There are two items involved. Handle of connecting item (usually
    a line) is being dragged by an user towards another item (item in
    short). Port of an item is found by the tool and connection is
    established by creating a constraint between line's handle and item's
    port.
    """

    def glue(self, item, handle, vpos):
        """
        Perform a small glue action to ensure the handle is at a proper
        location for connecting.
        """

        if item.from_handle() is handle:
            glue_distance = 5.0 / ((item.hierarchy_level + 1) * 2)
        else:
            glue_distance = 5.0 / (item.hierarchy_level * 2)

        if self.motion_handle:
            return self.motion_handle.glue(vpos, glue_distance)
        else:
            return HandleInMotion(item, handle, self.view).glue(vpos, glue_distance)

    def connect(self, item, handle, vpos):
        """
        Connect a handle of a item to connectable item.

        Connectable item is found by `ConnectHandleTool.glue` method.

        :Parameters:
         item
            Connecting item.
         handle
            Handle of connecting item.
         vpos
            Position to connect to (or near at least)
        """
        connector = Connector(item, handle)

        # find connectable item and its port
        sink = self.glue(item, handle, vpos)

        # no new connectable item, then diconnect and exit
        if sink:
            connector.connect(sink)
        else:
            cinfo = item.canvas.get_connection(handle)
            if cinfo:
                connector.disconnect()

    def on_button_release(self, event):
        view = self.view
        item = self.grabbed_item
        handle = self.grabbed_handle
        try:
            if handle and handle.connectable:
                self.connect(item, handle, (event.x, event.y))
        finally:
            return super(MyConnectHandleTool, self).on_button_release(event)