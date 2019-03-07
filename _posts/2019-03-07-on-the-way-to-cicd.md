---
layout: post
title:  "On the way to CI/CD: Python 3 problems fixed with the help of tox"
date:   2019-03-07 14:54:00 +0100
categories: python gtk release
author: Franz Steinmetz
---

RAFCON has still no fully-stacked continuous integration and continuous delivery (CI/CD) system.
With the [latest release `0.13.7`][latest] and the integration of [tox], we are one step closer.

What is most important for you as user:
RAFCON can finally be installed under Python 3 without any problems!
We are really sorry what there have been issues in the previous releases and that it took so long to fix them.
This is also because there was little feedback from the community.
So please never hesitate to contact us if you have any problems with RAFCON.

How does this relate to tox?
Tox helps us to run the RAFCON tests easily under different Python interpreters and in isolated environments.
We also started automating these test runs with Jenkins.

We hope that we extend our CI/CD system soon, which would allow us to release more often.
Currently, releasing a new version requires [a lot of manual steps][release steps] and is thus time consuming.

[latest]: https://pypi.org/project/rafcon/0.13.7/
[tox]: https://tox.readthedocs.io/
[release steps]: https://rafcon.readthedocs.io/en/latest/development/development.html#steps-for-releasing
