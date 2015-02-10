#!/bin/bash

export PYTHONPATH=$PYTHONPATH:$1:$1/lib/python2.7/site-packages/:$1/lib/python2.7/site-packages/gtk-2.0
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$1/lib/pkgconfig

mkdir -p $1
cd $1

# PyGObject 2.28.3
wget https://pypi.python.org/packages/source/P/PyGObject/pygobject-2.28.3.tar.bz2#md5=aa64900b274c4661a5c32e52922977f9
tar xjf pygobject-2.28.3.tar.bz2
cd pygobject-2.28.3
patch gio/gio-types.defs -i /volume/USERSTORE/brun_sb/public/gio-types.patch
./configure --prefix=$1 --disable-introspection
make
make install
cd ..
ln -sf pygobject-2.28.3/gobject gobject
ln -sf pygobject-2.28.3/glib glib
ln -sf pygobject-2.28.3/gio gio
ln -sf pygobject-2.28.3/gi gi

# PyGTK 2.24
wget http://ftp.gnome.org/pub/GNOME/sources/pygtk/2.24/pygtk-2.24.0.tar.gz
tar xzf pygtk-2.24.0.tar.gz
cd pygtk-2.24.0
./configure --prefix=$1
make
make install
cd ..

# Pycairo 1.10.0
wget http://cairographics.org/releases/py2cairo-1.10.0.tar.bz2
tar xjf py2cairo-1.10.0.tar.bz2
cd py2cairo-1.10.0
autoreconf -f -i -Wall,no-obsolete
./configure --prefix=$1
make
make install
cd ..

# gtkmvc 1.99.1
wget http://sourceforge.net/projects/pygtkmvc/files/pygtkmvc/1.99.1/python-gtkmvc-1.99.1.tar.gz/download 
tar xzf python-gtkmvc-1.99.1.tar.gz
cd python-gtkmvc-1.99.1
python setup.py install --prefix=$1
cd ..

# enum34 1.0.4
wget https://pypi.python.org/packages/source/e/enum34/enum34-1.0.4.tar.gz#md5=ac80f432ac9373e7d162834b264034b6
tar xzf enum34-1.0.4.tar.gz
cd enum34-1.0.4
python setup.py install --prefix=$1
cd ..

# PyOpenGL 3.1.0
wget https://pypi.python.org/packages/source/P/PyOpenGL/PyOpenGL-3.1.0.tar.gz#md5=0de021941018d46d91e5a8c11c071693
tar xzf PyOpenGL-3.1.0.tar.gz
cd PyOpenGL-3.1.0
python setup.py install --prefix=$1
cd ..

# logilab-common 0.63.2
wget https://pypi.python.org/packages/source/l/logilab-common/logilab-common-0.63.2.tar.gz#md5=2bf4599ae1f2ccf4603ca02c5d7e798e
tar xzf logilab-common-0.63.2.tar.gz
cd logilab-common-0.63.2
python setup.py install --prefix=$1
cd ..

# astroid 1.3.4
wget https://pypi.python.org/packages/source/a/astroid/astroid-1.3.4.tar.gz#md5=24e89453422bc39b0e1a9e4d7e0a1b0f
tar xzf astroid-1.3.4.tar.gz
cd astroid-1.3.4
python setup.py install --prefix=$1
cd ..

# pylint 1.4.1
wget https://pypi.python.org/packages/source/p/pylint/pylint-1.4.1.tar.gz#md5=df7c679bdcce5019389038847e4de622
tar xzf pylint-1.4.1.tar.gz
cd pylint-1.4.1
python setup.py install --prefix=$1
cd ..

# pygtkglext 1.1.0
wget http://sourceforge.net/projects/gtkglext/files/pygtkglext/1.1.0/pygtkglext-1.1.0.tar.gz/download
tar xzf pygtkglext-1.1.0.tar.gz
cd pygtkglext-1.1.0
./configure --prefix=$1
make
make install
cd ..

# PyGTKSourceView 2.10.1
wget http://ftp.gnome.org/pub/gnome/sources/pygtksourceview/2.10/pygtksourceview-2.10.1.tar.bz2
tar xjf pygtksourceview-2.10.1.tar.bz2
cd pygtksourceview-2.10.1
# Make sure, the PGK_CONFIG_PATH is correctly set to $1/lib/pkgconfig
./configure --prefix=$1
make
make install
cd ..


