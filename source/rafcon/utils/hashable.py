import hashlib


class Hashable(object):
    @staticmethod
    def update_hash_from_dict(hash, value):
        if isinstance(value, Hashable):
            value.update_hash(hash)
        elif isinstance(value, dict):
            for v in value.itervalues():
                Hashable.update_hash_from_dict(hash, v)
        else:
            hash.update(str(value))

    def update_hash(self, obj_hash):
        raise NotImplementedError()

    def mutable_hash(self):
        obj_hash = hashlib.sha256()
        self.update_hash(obj_hash)
        return obj_hash
