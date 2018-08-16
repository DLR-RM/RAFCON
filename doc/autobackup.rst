Auto Backup
===========

There are session restore and auto backup features on state machine level
which are enabled by default and which can be used to backup and restore your work.
These features can be enabled/disabled by the parameter
``SESSION_RESTORE_ENABLED`` and ``AUTO_BACKUP_ENABLED`` in the GUI config.

The session restore uses the backup of state machines and other GUI
information to restore your open state machine tabs and respective
selection situation.

If the state machine auto backup is enabled RAFCON creates temporary backups
of your open state machines.
You can find these after a crash of RAFCON on your computer in
``$RUNTIME_BACKUP_PATH = /tmp/rafcon-$USER/$PID/runtime_backup/``.
``$USER`` is your user name and ``$PID`` was/is the process id of your
RAFCON instance.
If a state machine hasn't been saved before, it will be
located at ``$RUNTIME_BACKUP_PATH/not_stored_$SM_ID``, whereby ``$SM_ID`` is the ID of the state machine.
If your state-machine has already been stored, the state machine backup path is
``$RUNTIME_BACKUP_PATH/$SM_BASE_PATH``, whereas ``$SM_BASE_PATH`` is the
path to your state machine.

The automatic backup can either be disabled or enabled. If enabled either a fixed forced or
dynamic interval can be set. Respective parameters are described in :ref:`Configuration` and start with
``AUTO_BACKUP_*``.

In case of a fixed forced interval it is checked every duration ``T`` if
there was a change to the state-machine. This means that a modification can
maximal not been backuped for ``T``. ``T`` is specified by ``AUTO_BACKUP_FORCED_STORAGE_INTERVAL``.

In case of dynamic auto-backup it is tried to avoid user disturbances by
waiting for a time-interval ``T*``, within which the user has not modified
the state-machine, to trigger the auto-backup while still using ``T`` as
a hard limit. This means that a modification is possibly backup-ed every ``T*``
and forced after ``T``. ``T*`` is specified by the config value ``AUTO_BACKUP_DYNAMIC_STORAGE_INTERVAL``.


Auto Recovery
-------------

With the release 0.7.5 lock files for state machines and rafcon
instances are introduced in the ``$RUNTIME_BACKUP_PATH``. State machines
which were not proper stored can be identified and recovered on a formal
way (dialog window) if the parameters ``AUTO_RECOVERY_LOCK_ENABLED`` and
``AUTO_RECOVERY_CHECK`` are set to ``True``.
By default those parameters are set ``False``. In more detail
``AUTO_RECOVERY_LOCK_ENABLED`` results in the creation of a lock file and
``AUTO_RECOVERY_CHECK=True`` triggers a check on lock files for the whole
``/tmp/rafcon-$USER`` folder and will offer optional recovery of
respective state machines. So it is possible to enable lock file generation
and only enable the check on lock files if explicitly needed.

An auto recovery of whole crashed sessions including their open state machines
and tab states is currently not supported. Nevertheless, sessons can be restored
if RAFCON was closed correctly and the config value ``SESSION_RESTORE_ENABLED`` is set to True.
