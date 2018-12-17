# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>


def check_if_dict_contains_object_reference_in_values(object_to_check, dict_to_check):
    """ Method to check if an object is inside the values of a dict.
    A simple object_to_check in dict_to_check.values() does not work as it uses the __eq__ function of the object
    and not the object reference.

    :param object_to_check: The target object.
    :param dict_to_check: The dict to search in.
    :return:
    """
    for key, value in dict_to_check.items():
        if object_to_check is value:
            return True
    return False
