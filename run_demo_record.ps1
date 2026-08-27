# Presentation demo: bring the stack up and record the small bag.
# Drive it from a second window with ./run_demo_teleop.ps1
#
#   ./run_demo_record.ps1                       # husarion_office, 1 rover
#   ./run_demo_record.ps1 office_world 2        # two rovers
param(
  [string]$World = "husarion_office",
  [int]$NumRobots = 1
)
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $PSCommandPath
$WslRepo = (wsl -d Ubuntu wslpath -u $Repo).Trim()
wsl -d Ubuntu bash -lc "chmod +x '$WslRepo/scripts/demo_teleop_record.sh' && '$WslRepo/scripts/demo_teleop_record.sh' $World $NumRobots"
