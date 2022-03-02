VERSION="1.44.0"

rm "dart-sass-"$VERSION"-linux-x64.tar.gz"
rm -r dart-sass
wget "https://github.com/sass/dart-sass/releases/download/"$VERSION"/dart-sass-"$VERSION"-linux-x64.tar.gz" &&
tar xvzf "dart-sass-"$VERSION"-linux-x64.tar.gz" &&
dart-sass/sass share/themes/RAFCON/sass/gtk.scss share/themes/RAFCON/gtk-3.0/gtk.css &&
dart-sass/sass share/themes/RAFCON/sass/gtk-dark.scss share/themes/RAFCON/gtk-3.0/gtk-dark.css