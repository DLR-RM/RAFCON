from math import sqrt


def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def point_on_line(point, line_start, line_end):
    length = dist(line_start, line_end)
    ds = length / 50.
    if -ds < (dist(line_start, point) + dist(point, line_end) - length) < ds:
        return True

    return False


def point_in_triangle(p, v1, v2, v3):
    def _test(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    b1 = _test(p, v1, v2) < 0.0
    b2 = _test(p, v2, v3) < 0.0
    b3 = _test(p, v3, v1) < 0.0

    return (b1 == b2) and (b2 == b3)