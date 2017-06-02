# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>


class classproperty(property):
    """Decorator for classes to allow class properties
    
    By default a @property method cannot at the same time by a @classmethod. This decorator serves as replacement for
    @property.
    """
    def __get__(self, cls, owner):
        return self.fget.__get__(None, owner)()
