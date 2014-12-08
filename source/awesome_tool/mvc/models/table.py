
class ColumnDescriptor:

    def __init__(self, no, key, type):
        self.no = no
        self.type = type
        self.key = key

class AttributesRowDescriptor:

    def __init__(self, no, key, name, editable=True, hidden=False):
        self.no = no
        self.key = key
        self.name = name
        self.editable = editable
        self.hidden = hidden


class TableDescriptor:

    def __init__(self, columns=None, rows=None):
        if columns is not None:
            assert isinstance(columns, list)
            self.columns = columns
            self._sort_columns()
        else:
            self.columns = []

        if rows is not None:
            assert isinstance(rows, list)
            self.rows = rows
            self._sort_rows()
        else:
            self.rows = []

    def add_column(self, column):
        assert isinstance(column, ColumnDescriptor)
        self.columns.append(column)
        self._sort_columns()

    def add_row(self, row):
        assert isinstance(row, AttributesRowDescriptor)
        self.rows.append(row)
        self._sort_rows()

    def get_column_types(self):
        return map(lambda c: c.type, self.columns)  # reduce columns to types

    def _sort_columns(self):
        self.columns = sorted(self.columns, key=lambda c: c.no)

    def _sort_rows(self):
        self.rows = sorted(self.rows, key=lambda r: r.no)