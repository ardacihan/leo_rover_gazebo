# SLAM + Nav2 costmaps + autonomous navigation in RViz
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$WslRepo = (wsl -d Ubuntu wslpath -u $Repo).Trim()
wsl -d Ubuntu bash -lc "chmod +x '$WslRepo/scripts/slam_nav2_wsl.sh' && '$WslRepo/scripts/slam_nav2_wsl.sh'"
