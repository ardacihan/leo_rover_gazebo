# Start GPU Gazebo sim (Docker + WSLg)
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$WslRepo = (wsl -d Ubuntu wslpath -u $Repo).Trim()
wsl -d Ubuntu bash -lc "chmod +x '$WslRepo/scripts/sim_gpu_wsl.sh' && '$WslRepo/scripts/sim_gpu_wsl.sh'"
