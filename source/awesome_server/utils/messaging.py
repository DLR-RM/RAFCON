import uuid
import hashlib


def create_send_message(message):
    checksum = create_unique_checksum(message)
    return checksum + message


def create_unique_checksum(message):
    salt = uuid.uuid4().hex
    return hashlib.md5(salt.encode() + message.encode()).hexdigest()[:6] + ":" + salt


def check_checksum(message):
    checksum, salt = message[:39].split(":")
    message = message[39:]
    current_checksum = hashlib.md5(salt.encode() + message.encode()).hexdigest()[:6]
    return checksum == current_checksum