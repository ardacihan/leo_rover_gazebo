# Autonomous frontier exploration (explore_lite) — needs sim + SLAM/Nav2 running
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$WslRepo = (wsl -d Ubuntu wslpath -u $Repo).Trim()
wsl -d Ubuntu bash -lc "chmod +x '$WslRepo/scripts/explore_wsl.sh' && '$WslRepo/scripts/explore_wsl.sh'"
