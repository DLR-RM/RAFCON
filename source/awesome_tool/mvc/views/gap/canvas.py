from gaphas.canvas import Canvas
from gaphas.solver import Projection
from gaphas.decorators import nonrecursive, async, PRIORITY_HIGH_IDLE

from awesome_tool.mvc.views.gap.state import StateView, NameView
from awesome_tool.mvc.views.gap.constraint import KeepRectangleWithinConstraint


class MyCanvas(Canvas):

    @async(single=True, priority=PRIORITY_HIGH_IDLE)
    def update(self):
        """
        Update the canvas, if called from within a gtk-mainloop, the
        update job is scheduled as idle job.
        """
        if (len(self._dirty_matrix_items) == 1 and isinstance(list(self._dirty_matrix_items)[0], StateView) and
                list(self._dirty_matrix_items)[0].moving):
            self.update_now_moving()
        else:
            self.update_now()

    @nonrecursive
    def update_now_moving(self):
        """
        Peform an update of the items that requested an update.
        """

        if self._dirty_index:
            self.update_index()
            self._dirty_index = False

        sort = self.sort
        extend_dirty_items = self._extend_dirty_items

        # perform update requests for parents of dirty items
        dirty_items = self._dirty_items
        for item in set(dirty_items):
            dirty_items.update(self._tree.get_ancestors(item))

        # order the dirty items, so they are updated bottom to top
        dirty_items = sort(self._dirty_items, reverse=True)

        self._dirty_items.clear()

        try:
            cr = self._obtain_cairo_context()

            # allow programmers to perform tricks and hacks before item
            # full update (only called for items that requested a full update)
            self._pre_update_items(dirty_items, cr)

            # recalculate matrices
            dirty_matrix_items = self.update_matrices(self._dirty_matrix_items)
            self._dirty_matrix_items.clear()

            self.update_outer_constraints(dirty_matrix_items)

            # no matrix can change during constraint solving
            assert not self._dirty_matrix_items, 'No matrices may have been marked dirty (%s)' % (self._dirty_matrix_items,)

            # item's can be marked dirty due to external constraints solving
            extend_dirty_items(dirty_items)

            assert not self._dirty_items, 'No items may have been marked dirty (%s)' % (self._dirty_items,)

            # normalize items, which changed after constraint solving;
            # store those items, whose matrices changed
            normalized_items = self._normalize(dirty_items)

            # recalculate matrices of normalized items
            dirty_matrix_items.update(self.update_matrices(normalized_items))

            # ensure constraints are still true after normalization
            self._solver.solve()

            extend_dirty_items(dirty_items)

            assert not self._dirty_items, 'No items may have been marked dirty (%s)' % (self._dirty_items,)

            self._post_update_items(dirty_items, cr)

        except Exception, e:
            print 'Error while updating canvas', e

        assert len(self._dirty_items) == 0 and len(self._dirty_matrix_items) == 0, \
                'dirty: %s; matrix: %s' % (self._dirty_items, self._dirty_matrix_items)

        self._update_views(dirty_items, dirty_matrix_items)

    def update_outer_constraints(self, items):
        assert len(items) == 1
        request_resolve = self._solver.request_resolve
        for item in items:
            for p in item._canvas_projections:
                p0 = self.get_var(p[0])
                p1 = self.get_var(p[1])
                p0_cons_to_remove = set()
                p1_cons_to_remove = set()

                for cons in p0._constraints:
                    if isinstance(cons, KeepRectangleWithinConstraint) and cons.child is not item:
                        p0_cons_to_remove.add(cons)
                for cons in p1._constraints:
                    if isinstance(cons, KeepRectangleWithinConstraint) and cons.child is not item:
                        p1_cons_to_remove.add(cons)
                for rem in p0_cons_to_remove:
                    p0._constraints.remove(rem)
                for rem in p1_cons_to_remove:
                    p1._constraints.remove(rem)

                request_resolve(p0, projections_only=True)
                request_resolve(p1, projections_only=True)
        self._solver.solve()

    def get_var(self, projection):
        while isinstance(projection, Projection):
            projection = projection.variable()
        return projection

    def update_constraints(self, items):
        """
        Update constraints. Also variables may be marked as dirty before the
        constraint solver kicks in.
        """
        # request solving of external constraints associated with dirty items
        request_resolve = self._solver.request_resolve
        for item in items:
            for p in item._canvas_projections:
                request_resolve(p[0], projections_only=True)
                request_resolve(p[1], projections_only=True)
        # solve all constraints
        self._solver.solve()

    def update_matrices(self, items):
        return self._update_matrices_moving(items)

    def _update_matrices_moving(self, items, moving=False):
        """
        Recalculate matrices of the items. Items' children matrices are
        recalculated, too.

        Return items, which matrices were recalculated.
        """
        changed = set()
        for item in items:
            parent = self._tree.get_parent(item)
            if parent is not None and parent in items:
                # item's matrix will be updated thanks to parent's matrix
                # update
                continue

            if isinstance(item, (StateView, NameView)) and item.moving:
                self.update_matrix(item, parent)
                changed.add(item)

                # changed_children = self._update_matrices_moving(set(self.get_children(item)), True)
                # changed.update(changed_children)
            elif not moving:
                self.update_matrix(item, parent)
                changed.add(item)

                changed_children = self._update_matrices_moving(set(self.get_children(item)))
                changed.update(changed_children)

        return changed