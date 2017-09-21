Steps to perform, when releasing a new version of RAFCON:

1. Decide about the new version number

  Should the release be a patch or minor release? Or even a major release? Check the latest version number and adjust
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

  Before creating the required state machine, RAFCON must be correctly installed so its version can be determined.
  For this, navigate to the RAFCON root folder (where ``setup.py`` resides) and call

  .. code:: bash

     $ python setup.py develop --user

  Then, open the state machine in ``[project directory]/tests/assets/unit_test_state_machines/[latest version
  number]`` and save it to ``[project directory]/source/test_scripts/backwards_compatibility/[new version number]``.
  Commit your changes.

5. Check tests

  Run all tests and verify that they do all pass. If not, fix them! Also check the BuildBot. Commit your changes.

6. Check the changelog

  Open ``[project directory]/doc/Changelog.rst`` and verify that all changes are included within the correct version
  number. Compare with :code:`git lg` and the latest closed issues on GitHub. Commit your changes.

7. Apply the version number

  Update the version number in ``[project directory]/VERSION``.
  Commit your changes.

8. Merge to master

  When everything is prepared, you can merge the release branch into the master branch:

  .. code:: bash

     $ git checkout master
     $ git merge release-[new version number]]


9. Do the release

  Make sure, everything is pushed (:code:`git push`). Then copy the release notes for the specific version into a
  temporary file, e.g. ``/tmp/release_notes.rst``. Finally do:

  .. code:: bash

     $ cd [projet directory]]
     $ rmpm_do release --domain software.common --version [new version number] -F /tmp/release_notes.rst

10. Merge to develop

  Merge all changes back into the develop branch:

  .. code:: bash

     $ git checkout develop
     $ git merge release-[new version number]]
