# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

import hashlib


class Hashable(object):
    @staticmethod
    def update_hash_from_dict(obj_hash, value):
        """Updates an existing hash object with another Hashable or a dictionary

        :param obj_hash: The hash object (see Python hashlib documentation)
        :param value: The value that should be added to the hash (can be another Hashable or a dictionary)
        """
        if isinstance(value, Hashable):
            value.update_hash(obj_hash)
        elif isinstance(value, dict):
            for v in value.itervalues():
                Hashable.update_hash_from_dict(obj_hash, v)
        else:
            obj_hash.update(str(value))

    def update_hash(self, obj_hash):
        """Should be implemented by derived classes to update the hash with their data fields

        :param obj_hash: The hash object (see Python hashlib)
        """
        raise NotImplementedError()

    def mutable_hash(self, obj_hash=None):
        """Creates a hash with the (im)mutable data fields of the object

        Example:
            >>> my_obj = type("MyDerivedClass", (Hashable,), { "update_hash": lambda self, h: h.update("RAFCON") })()
            >>> my_obj_hash = my_obj.mutable_hash()
            >>> print('Hash: ' + my_obj_hash.hexdigest())
            Hash: c8b2e32dcb31c5282e4b9dbc6a9975b65bf59cd80a7cee66d195e320484df5c6

        :param obj_hash: The hash object (see Python hashlib)
        :return: The updated hash object
        """
        if obj_hash is None:
            obj_hash = hashlib.sha256()
        self.update_hash(obj_hash)
        return obj_hash
