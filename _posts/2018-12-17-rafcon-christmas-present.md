---
layout: post
title:  "A christmas present from the RAFCON developers"
date:   2018-12-17 17:52:00 +0100
categories: python gtk release
author: Franz Steinmetz
---

We have released a shiny new minor version of RAFCON, `0.13.0`. The biggest news: You can now use RAFCON with both Python 2 and 3, plus the GUI is now implemented with GTK+3 (instead of GTK+ 2).

Thus, you can now freely choose your preferred Python interpreter. Thanks to the ``future``, all interpreters share the same code base.

With GTK+ 3, styling the GUI is now much easier for us, as we can use SASS and CSS. We used the new freedom to provide you a second theme, namely a light theme (see image below)! Out thanks here also goes to [Interaktionswerk][interaktionswerk], the company that provides the design templates for us. GTK+ 3 allows for better integration with the window manager, as it e.g. offers a `HeaderBar` and custom window decorations, while still being able to resize the window along the borders. GTK+ 3 uses `gobject-introspection`, reducing the amount of dependencies. We also decided on dropping the old graphical editor that was using OpenGL.

![The new light theme of RAFCON](images/news/RAFCON-0.13.0.png){:class="img-responsive"}

Of course, there are many more new features. For example, RAFCON now install a `*.desktop` file, so that it is added to the application menu.

In the [RAFCON repository][repo], you can now also find a `CITATION.cff` file, which provides all the information necessary (including a DOI) to cite our software, e.g. in a paper. 

Furthermore, lots of bugs have been fixed!

So do not hesitate and update your RAFCON installation. If you find any bugs, please file them to our [issue tracker][issuetracker].

[interaktionswerk]: https://interaktionswerk.de/
[repo]: https://github.com/DLR-RM/RAFCON/
[issuetracker]: https://github.com/DLR-RM/RAFCON/issues
