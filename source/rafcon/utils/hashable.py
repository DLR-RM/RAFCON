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

    def mutable_hash(self, obj_hash=hashlib.sha256()):
        """Creates a hash with the (im)mutable data fields of the object

        Example:
            >>> my_hash_obj = Hashable().mutable_hash()
            >>> print('Hash: ' + my_hash_obj.hexdigest())

        :param obj_hash: The hash object (see Python hashlib)
        :return: The updated hash object
        """
        self.update_hash(obj_hash)
        return obj_hash
