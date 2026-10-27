# start_gui.ps1 [-RobotIp NEW_IP] : copy the newest places list from the robot, then open the guide GUI.
# Run: powershell -ExecutionPolicy Bypass -File $env:USERPROFILE\lab\start_gui.ps1 [-RobotIp 10.25.x.x]
param([string]$RobotIp)
$lab = "$env:USERPROFILE\lab"
if ($RobotIp) {
  Set-Content -Encoding ascii "$lab\robot_ip.txt" $RobotIp
  $cfg = "$env:USERPROFILE\.ssh\config"
  (Get-Content $cfg) -replace '^(\s+HostName\s+).*$', "`${1}$RobotIp" | Set-Content -Encoding ascii $cfg
  Write-Host "robot IP set to $RobotIp"
}
scp -q -o BatchMode=yes -o ConnectTimeout=5 limo:guide_ws/places_lab.yaml "$lab\" 2>$null
if ($LASTEXITCODE -eq 0) { Write-Host "places copied from the robot" }
else { Write-Host "places not copied (robot off, SSH key not installed, or no places yet): using the last copy" }
py "$lab\lab_gui_win.py"
