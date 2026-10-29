# setup_laptop.ps1 -RobotIp 10.25.x.x : one-time Windows laptop setup for the lab guide GUI (no ROS needed).
# Run from a clone of the repo:  powershell -ExecutionPolicy Bypass -File .\laptop\setup_laptop.ps1 -RobotIp <robot IP>
# Robot IP: on the robot screen, `ip -4 -o addr show wlan0` (address before the /).
# Does: Python packages + speech model check, copies the GUI files to %USERPROFILE%\lab, robot_ip.txt,
# SSH shortcut "limo" (user agilex) and SSH key on the robot (asks the robot password once; Limo: AGX).
param([Parameter(Mandatory = $true)][string]$RobotIp)
$ErrorActionPreference = "Continue"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$lab = "$env:USERPROFILE\lab"
Write-Host "== 1 Python packages"
if (-not (Get-Command py -ErrorAction SilentlyContinue)) { Write-Host "Python missing: winget install -e --id Python.Python.3.12, reopen PowerShell, run again"; exit 1 }
py --version
py -m pip install -q -r "$here\requirements.txt"
if ($LASTEXITCODE -ne 0) { Write-Host "pip install failed (see above)"; exit 1 }
py -c "from faster_whisper import WhisperModel; WhisperModel('base.en', device='cpu', compute_type='int8'); print('  OK   whisper model')"
Write-Host "== 2 GUI files -> $lab"
New-Item -ItemType Directory -Force $lab | Out-Null
Copy-Item "$here\lab_gui_win.py", "$here\start_gui.ps1" $lab -Force
Set-Content -Encoding ascii "$lab\robot_ip.txt" $RobotIp
Write-Host "  OK   lab_gui_win.py, start_gui.ps1, robot_ip.txt ($RobotIp)"
Write-Host "== 3 SSH shortcut 'limo' and key"
$ssh = "$env:USERPROFILE\.ssh"; New-Item -ItemType Directory -Force $ssh | Out-Null
$cfg = "$ssh\config"
if (-not (Test-Path $cfg) -or -not (Select-String -Path $cfg -Pattern '^Host limo$' -Quiet)) {
  Add-Content -Encoding ascii -Path $cfg -Value "`nHost limo`n HostName $RobotIp`n User agilex`n ServerAliveInterval 30"
} else {
  (Get-Content $cfg) -replace '^(\s+HostName\s+).*$', "`${1}$RobotIp" | Set-Content -Encoding ascii $cfg
}
if (-not (Test-Path "$ssh\id_ed25519.pub")) { ssh-keygen -t ed25519 -N '""' -f "$ssh\id_ed25519" | Out-Null }
ssh -o BatchMode=yes -o ConnectTimeout=5 limo "echo key ok" 2>$null
if ($LASTEXITCODE -ne 0) {
  Write-Host "  robot password once (Limo: AGX):"
  Get-Content "$ssh\id_ed25519.pub" | ssh limo "mkdir -p ~/.ssh && tr -d '\r' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
  ssh -o BatchMode=yes limo "echo key ok"
}
Write-Host "`nLAPTOP OK. Start the GUI (robot side running): powershell -ExecutionPolicy Bypass -File $lab\start_gui.ps1"
