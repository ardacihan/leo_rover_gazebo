# Drive the rover during a demo recording (run ./run_demo_record.ps1 first).
#
#   ./run_demo_teleop.ps1        # drive leo1
#   ./run_demo_teleop.ps1 2      # two rovers, switch with the 1 / 2 keys
param(
  [int]$NumRobots = 1
)
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $PSCommandPath
$WslRepo = (wsl -d Ubuntu wslpath -u $Repo).Trim()
Write-Host "Click the WSL window: W/A/S/D to drive, SPACE stop, 1/2 pick rover, Q quit."
wsl -d Ubuntu bash -lc "chmod +x '$WslRepo/scripts/demo_teleop_wsl.sh' && '$WslRepo/scripts/demo_teleop_wsl.sh' $NumRobots"
