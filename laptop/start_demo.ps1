# start_demo.ps1 : the whole demo in one command: robot side (clean start + guide_start.sh in tmux "guide"), wait until ready, then the GUI.
# Run: powershell -ExecutionPolicy Bypass -File $env:USERPROFILE\lab\start_demo.ps1   (or the "Lab Guide Demo" desktop icon)
$lab = "$env:USERPROFILE\lab"
Read-Host "Robot ON, on the Home tape facing front? Press Enter to start"
scp -q -o BatchMode=yes -o ConnectTimeout=5 "$lab\demo_robot.sh" limo:guide_ws/scripts/demo_robot.sh
if ($LASTEXITCODE -ne 0) { Write-Host "robot not reachable (ssh limo): check power, Wi-Fi, IP in robot_ip.txt"; Read-Host "Enter to close"; exit 1 }
Write-Host "starting the robot side (about 40-60 s)..."
ssh -o BatchMode=yes limo "bash guide_ws/scripts/demo_robot.sh"
if ($LASTEXITCODE -ne 0) { Read-Host "robot side not ready: Enter to close"; exit 1 }
& "$lab\start_gui.ps1"
