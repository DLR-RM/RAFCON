General Git commit conventions
""""""""""""""""""""""""""""""

Git is used as our version control system. Please keep in the following points in mind when committing to the
`repository <https://github.com/DLR-RM/RAFCON>`__:

-  if you are not a core developer, all changes must be bundles in pull requests
-  therefore, a new branch is required for new features and bug fixes
-  try to keep pull requests small, this eases the review and will fasten the merge of the branch
-  before filing a pull request, make sure that all tests are passed
-  new features require new unit tests
-  each functional/logical change gets its own commit (try to keep them small, too)
-  no excessive use of ``logger.debug`` outputs (in commits) and never commit ``print`` commands


Git commit messages
"""""""""""""""""""

When looking at the Git commit history, it should be easy to see what changes have been performed. This is why it is
important to have good commit messages.

**What to do:**

-  Use imperative (`Add file …`)
-  First line is the caption of the commit (should be less than 50 chars)

   -  summarizes the commit
   -  mentions which code is affected (e.g. by stating the changes module/class)

-  Second line is blank
-  Following lines give a longer description and list changes in detail (use "- " or "* " as bullet points)

   -  Why is the change necessary
   -  How does it address the issue
   -  Possible side effects
   -  Give Issue/Feature-ID of `GitHub Issues <https://github.com/DLR-RM/RAFCON/issues>`__

      -  ``Clos[e[s]]|Fix[e[s]]|Resolve[e[s]] #1234567`` – use one of the keywords to automatically close an issue
      -  ``Relates to issue #1234567`` – state issue id also when the issue is not supposed to be closed

**What not to do:**

-  Try to avoid using the ``-m <msg>``/``--message=<msg>`` flag to ``git commit``
-  Do not assume that the reader knows the details about bugs, features or previous commits
-  Neither assume that the reader looks at the commited code


**Setting up your system**

Add this line to your ``~/.vimrc`` to add spell checking and automatic wrapping at the recommended 72 columns to you commit messages.

``autocmd Filetype gitcommit setlocal spell textwidth=72``

**Explaining examples**

This is a good example for acommit message:

.. code-block:: none

  Example: Capitalized, short (<50 chars) summary

  More detailed explanatory text. Wrap it to about 72 characters. Think
  of first line as the subject of an email and the rest of the text as
  the body. The blank line separating the summary from the body is
  critical (unless you omit the body entirely); tools like rebase can get
  confused if you run the two together.

  Further paragraphs come after blank lines.

  - Dash '-' and asterisk '*' are both fine, followed by a space
  - Use a hanging indent

Th following link shows some bad examples: https://wiki.openstack.org/wiki/GitCommitMessages#Some_examples_of_bad_practice

**Sources**

- http://robots.thoughtbot.com/5-useful-tips-for-a-better-commit-message
- http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html
- https://wiki.openstack.org/wiki/GitCommitMessages
- https://help.github.com/articles/closing-issues-via-commit-messages/
