#!/bin/bash

cur_dir=`pwd`

mkdir -p $1
cd $1

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$1/lib:$1/lib64/
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$1/lib/pkgconfig
echo $LD_LIBRARY_PATH

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
MACHINE=$(uname -m)
if [ $MACHINE = "i686" ]; then
    export CFLAGS="-march=i486"
fi
export LIBFFI_CFLAGS=-I$1/lib/libffi-3.0.11/include
export LIBFFI_LIBS="-L$1/lib64 -lffi"
./configure --prefix=$1
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
./configure --prefix=$1
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
