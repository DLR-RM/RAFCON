#!/bin/bash
# The following libraries are going to be installed:
# libffi 3.2.1
# GLib 2.42.1
# gobject introspection 1.42.0
# Cairo 1.11.4
# GTKSourceView 2.10.5

cur_dir=`pwd`

SYSTEM=$(uname -s)
if [ $SYSTEM = "Linux" ]; then 
    MACHINE=$(uname -m)
fi
if [ $MACHINE = "i686" ]; then 
    echo "$SYSTEM $MACHINE"
elif [ $MACHINE = "x86_64" ]; then 
    echo "$SYSTEM $MACHINE"
else
    echo "Error: Unsupported architecture: $SYSTEM $MACHINE"
    exit 0
fi

mkdir -p $1
cd $1

export PKG_CONFIG_PATH=""
export PYTHONPATH=""

source /volume/USERSTORE/lehn_pt/ros/indigo/setup.bash

export LD_LIBRARY_PATH=$1/lib:$1/lib64:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$1/lib/pkgconfig:$PKG_CONFIG_PATH
echo "LD_LIBRARY_PATH $LD_LIBRARY_PATH"
echo "PKG_CONFIG_PATH $PKG_CONFIG_PATH"
echo "PYTHONPATH $PYTHONPATH"
echo "SYS_LIB_PATH $1"
echo "PY_LIB_PATH $2"

# libffi 3.2.1
wget ftp://sourceware.org/pub/libffi/libffi-3.2.1.tar.gz
tar xzf libffi-3.2.1.tar.gz
cd libffi-3.2.1
./configure --prefix=$1
make
make install
cd ..

# GLib 2.42.1
wget ftp://ftp.gnome.org/pub/gnome/sources/glib/2.42/glib-2.42.1.tar.xz
tar xf glib-2.42.1.tar.xz 
cd glib-2.42.1
export LIBFFI_CFLAGS=-I$1/lib/libffi-3.0.11/include
if [ $MACHINE = "i686" ]; then
    export LIBFFI_LIBS="-L$1/lib -lffi"
    ./configure --prefix=$1 CFLAGS="-march=i686" CXXFLAGS="-march=i686"
elif [ $MACHINE = "x86_64" ]; then
    export LIBFFI_LIBS="-L$1/lib64 -lffi"
    ./configure --prefix=$1
fi
make
make install
cd ..

# gobject introspection 1.42.0
wget http://ftp.gnome.org/pub/gnome/sources/gobject-introspection/1.42/gobject-introspection-1.42.0.tar.xz
tar xf gobject-introspection-1.42.0.tar.xz 
cd gobject-introspection-1.42.0
./configure --prefix=$1
make
make install
cd ..

# Cairo 1.11.4
wget http://cairographics.org/snapshots/cairo-1.11.4.tar.gz
tar xzf cairo-1.11.4.tar.gz
cd cairo-1.11.4
if [ $MACHINE = "i686" ]; then
    ./configure --prefix=$1 LDFLAGS="-L/usr/lib -lpixman-1"
elif [ $MACHINE = "x86_64" ]; then
    ./configure --prefix=$1
fi
make
make install
cd ..

# GTKSourceView 2.10.5
wget http://ftp.gnome.org/pub/gnome/sources/gtksourceview/2.10/gtksourceview-2.10.5.tar.gz 
tar xzf gtksourceview-2.10.5.tar.gz 
cd gtksourceview-2.10.5
./configure --prefix=$1 --enable-glade-catalog
make
make install


cd $cur_dir

set -- "$2"
source ./python_lib_install.sh
