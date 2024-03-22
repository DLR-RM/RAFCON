Steps to perform, when releasing a new version of RAFCON (this section is only for releasing the tool inside our
institute):

1. Decide about the new version number

  Should the release be a patch or minor release? Or even a major release? Check the latest version number and adjust it
  appropriately.

2. Create a release Branch

  Create a new branch from the latest commit you want to include into the new release. Optionally, if there are new
  untested feature, which you don't want to include into the release, first go back to a previous commit:

  .. code:: bash

     $ git checkout [hash]

  Then, create the new branch and check it out:

  .. code:: bash

     $ git checkout -b release-[new version number]

3. Fix relevant issues *(optional)*

  Within your new branch, you shouldn't integrate any new features, but only solve issues. The ``develop`` and
  ``feature-xy`` branches should be used for new features. Take your time to ensure that the most critical issues are
  fixed and RAFCON is running stable.

4. Create a test state machine for the new version *(only minor/major releases)*

  For each minor release, a state machine must be created to ensure backwards compatibility using a special test.

  Therefore, open the state machine in ``[project
  directory]/tests/assets/unit_test_state_machines/backward_compatibility/[latest version
  number]`` and save it to the same folder with the correct version as library name.
  Commit your changes.

5. Check tests

  Run all tests and verify that they do all pass. If not, fix them! Also check the BuildBot. Commit your changes.

6. Check the changelog

  Open ``[project directory]/doc/Changelog.rst`` and verify that all changes are included within the correct version
  number. Compare with :code:`git log` and the latest closed issues on GitHub. Commit your changes.

7. Build style files

  Build ``*.css`` files from ``*.scss`` files.

  .. code:: bash

     $ ./compile_scss.sh
     $ git add share/themes/RAFCON/gtk-3.0/*.css --force

8. Apply the version number

  1. If the dev dependencies have not yet been installed via pdm, then run ``pdm install --dev --no-editable``
  2. Update the version number by running ``pdm run bump2version [major / minor / or patch]``
  3. Update the ``date-released`` in ``[project directory]/CITATION.cff``.
  4. Run ``cffconvert --format zenodo --outfile .zenodo.json`` (see `"Making software citable" <https://guide.esciencecenter.nl/#/best_practices/documentation?id=software-citation>`__, requires Python 3)
  5. Commit and push your changes.

9. Merge to master

  When everything is prepared, you can merge the release branch into the master branch:

  .. code:: bash

     $ git push release-[new version number]
     $ git checkout master
     $ git pull master
     $ git merge release-[new version number]
     $ git push

10. Merge to develop

  Merge all changes back into the develop branch:

  .. code:: bash

     $ git checkout develop
     $ git pull
     $ git merge release-[new version number]]
     $ git push

11. Publish new release to PyPi

  Create a new distribution file and publish it on PyPi:

  .. code:: bash

     $ rm dist/*
     $ pdm build
     $ twine upload dist/*

12. Publish to GitHub

  Publish the changes to GitHub and GitHub Enterprise (assuming ``github`` is your GitHub remote name):

  .. code:: bash

     $ git push github
     $ git checkout master
     $ git push github

  Make a release on GitHub by navigating to `https://github.com/DLR-RM/RAFCON/releases/new`. Enter the new version
  number in the "Tag version" field. Optioanlly add a release title and decription. Click "Publish release".

13. Force build of GitHub pages

  Push an empty commit to the ``gh-pages`` branch:

  .. code:: bash

     $ git checkout gh-pages
     $ git commit -m 'rebuild pages' --allow-empty
     $ git push
     $ git push github
