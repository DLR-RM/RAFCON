# Copyright


def limit_string(text, max_length, seperator="&#x2026;"):
    from xml.sax.saxutils import escape
    if isinstance(text, basestring) and len(text) > max_length:
        half_length = (max_length - 1) / 2
        return escape(text[:half_length]) + seperator + escape(text[-half_length:])
    return escape(text)
