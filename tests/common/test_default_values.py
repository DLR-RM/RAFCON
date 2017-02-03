
# l = [[3, 4], (1,2,3), ("Rico", 2, 3)]
# t = (1,2,3)
# d = {'tuple': t, 'list': l}
import json
# s = json.dumps(d, sort_keys=True, indent=4)
# print s
# nd = json.loads(s)
# print d
# print nd
import ast
# print 50*"#"
# rr = l.__repr__()
# print rr, type(rr)
# r = ast.literal_eval(l.__repr__())
# print r
# r = ast.literal_eval('12')
# print r
# s = "Rico"
# print s, ast.literal_eval(s.__repr__())


# type(None)
v = None
r = ast.literal_eval(v.__repr__())
rr = str(v.__repr__())
print r, type(r), v == r, v.__repr__(), r == rr, rr, type(rr)


# elif data_type == unicode:
#    converted_value = unicode(string_value)
v = unicode('abcdef')
r = ast.literal_eval(v.__repr__())
rr = unicode(v.__repr__())
print r, v == r, v.__repr__(), r == rr, rr

# elif data_type == int:
#     converted_value = int(string_value)
v = 12
r = ast.literal_eval(v.__repr__())
rr = int(v.__repr__())
print r, type(r), v == r, v.__repr__(), r == rr, rr, type(rr)

# elif data_type == float:
#     converted_value = float(string_value)
v = 12.
r = ast.literal_eval(v.__repr__())
rr = float(v.__repr__())
print r, type(r), v == r, v.__repr__(), r == rr, rr, type(rr)

# elif data_type == bool:
#     converted_value = bool(literal_eval(string_value))
v = True
r = ast.literal_eval(v.__repr__())
rr = bool(v.__repr__())
print r, type(r), v == r, v.__repr__(), r == rr, rr, type(rr)

# elif data_type in (list, dict, tuple):
#     converted_value = literal_eval(string_value)
#     if type(converted_value) != data_type:
#         raise ValueError("Invalid syntax: {0}".format(string_value))
# elif data_type == object:
#     try:
#         converted_value = literal_eval(string_value)
#     except ValueError:
#         converted_value = literal_eval('"' + string_value + '"')
# elif isinstance(data_type, type):  # Try native type conversion
#     converted_value = data_type(string_value)
# elif isclass(data_type):  # Call class constructor
#     converted_value = data_type(string_value)
# else:
#     raise ValueError("No conversion from string '{0}' to data type '{0}' defined".format(
#         string_value, data_type))

from math import *

hidden_value = "this is secret"

def dangerous_function(filename):
    print open(filename).read()

#make a list of safe functions
safe_list = ['math','acos', 'asin', 'atan', 'atan2', 'ceil', 'cos', 'cosh', 'degrees', 'e', 'exp', 'fabs', 'floor', 'fmod', 'frexp', 'hypot', 'ldexp', 'log',
'log10', 'modf', 'pi', 'pow', 'radians', 'sin', 'sinh', 'sqrt', 'tan', 'tanh']
#use the list to filter the local namespace
safe_dict = dict([ (k, locals().get(k, None)) for k in safe_list ])
#add any needed builtins back in.
safe_dict['abs'] = abs

# user_func = raw_input("type a function: y = ")

command = "sum([1,2])"
command = "1>2"
import __builtin__
print __builtin__
print dir(__builtins__)
print safe_dict
# print eval(command)
print eval(command,{"__builtins__": None}, safe_dict)

import numpy
from numpy import *
# print dir(numpy)

a = numpy.ndarray((2,2))
    # ((2, 2), (2,1)))
print a
v = a
r = eval(v.__repr__())
# rr = numpy.array(v.__repr__())
print r, type(r), v == r, v.__repr__(), r == rr, rr, type(rr)
print type(r), r, ndim(r)
# print type(rr), rr, ndim(rr)
# print json.dumps(a)

u = unicode('ich')
s = str('ich')
print u == s, isinstance(u, basestring), isinstance(s, basestring)