
def limit_string(text, max_length, seperator="&#x2026;"):
    if isinstance(text, basestring) and len(text) > max_length:
        half_length = (max_length - 1) / 2
        return text[:half_length] + seperator + text[-half_length:]
    return text