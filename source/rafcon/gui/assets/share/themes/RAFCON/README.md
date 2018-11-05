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
