# Keyboard teleop (interactive WSL terminal)
$ErrorActionPreference = "Stop"
$Repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$WslRepo = (wsl -d Ubuntu wslpath -u $Repo).Trim()
Write-Host "Starting teleop in WSL - click that window and use W/A/S/D to drive."
wsl -d Ubuntu bash -lc "chmod +x '$WslRepo/scripts/teleop_wsl.sh' && '$WslRepo/scripts/teleop_wsl.sh'"
