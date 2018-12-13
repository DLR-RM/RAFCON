# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import object
from builtins import str
import sys
import hashlib


class Hashable(object):
    @staticmethod
    def update_hash_from_dict(obj_hash, object_):
        """Updates an existing hash object with another Hashable, list, set, tuple, dict or stringifyable object

        :param obj_hash: The hash object (see Python hashlib documentation)
        :param object_: The value that should be added to the hash (can be another Hashable or a dictionary)
        """
        if isinstance(object_, Hashable):
            object_.update_hash(obj_hash)
        elif isinstance(object_, (list, set, tuple)):
            if isinstance(object_, set):  # A set is not ordered
                object_ = sorted(object_)
            for element in object_:
                Hashable.update_hash_from_dict(obj_hash, element)
        elif isinstance(object_, dict):
            for key in sorted(object_.keys()):  # A dict is not ordered
                Hashable.update_hash_from_dict(obj_hash, key)
                Hashable.update_hash_from_dict(obj_hash, object_[key])
        else:
            obj_hash.update(Hashable.get_object_hash_string(object_))

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

    @staticmethod
    def get_object_hash_string(object_):
        obj_hash_string = str(object_)
        if sys.version_info >= (3,):
            obj_hash_string = obj_hash_string.encode('utf-8')
        return obj_hash_string
