# RAFCON Theme

This GTK+ 3 theme is loosely based (mainly structure-wise) on "Unbroken": https://github.com/thornjad/unbroken

## Compilation

Use `sass` to compile the SCSS file to CSS:

```commandline
> cd theme_root
> sass --scss --sourcemap=none --force --update sass:gtk-3.0
```

You can also run `sass` as background process that autmatically compiles the SCSS files as soon as changes are detected.
 This can for example be used in a PyCharm External Tool:
 
 ```commandline
> sass --scss --sourcemap=none --watch sass:gtk-3.0
 ```

This will generate the two files `gtk.css` and `gtk-dark.css` in the `gtk-3.0` folder.

## Render assets

The ``assets.svg`` file contains all assets as a vector graphic in a single file. The ``assets.txt`` file contains 
all names of all objects that will be rendered. Just execute the ``render-asstes.sh`` file to render them.

### Notes

* Assets will only be rendered, if they are not existing. So you have to delete them from the ``assets`` folder if 
you want to re-create the.
* Some named objects in the `assets.svg` file are groups. Do not ungroup these, otherwise the name and (sometimes) 
the transparency will be lost. You can use the `Edit paths` tool to access objects within a group.
* The file `RAFCON.json` is the font project file for [IcoMoon](https://icomoon.io), from which the RAFCON icon font is
created
