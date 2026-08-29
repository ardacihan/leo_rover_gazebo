# Aborted harness attempt

This directory is not a result and must not be used in comparisons. The
Windows-side command watchdog was accidentally set to 120 seconds. Its WSL
child survived briefly, then exited immediately after launching the explorers,
leaving the Docker container without the mission polling, save, and cleanup
path. The sensor preflight completed successfully, but no exploration result
was saved. The orphaned container was stopped at 23:15 Europe/Berlin.
