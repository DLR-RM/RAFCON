import hashlib


def create_checksum(header, message):
    return hashlib.md5(header + " " + message).hexdigest()[:6]


def check_checksum(header, message, checksum):
    return checksum == create_checksum(header, message)