# lab_check.ps1 [-Install] : laptop-side checks at the start of a lab session (guide 4.4 + 4.7).
# Robot answers ping; SSH key works (installs it once, password AGX); bridge ports 9090 / 8765 reachable.
# -Install also copies install_guide.sh from this folder to the robot (Windows line endings removed).
param([switch]$Install)
$lab = "$env:USERPROFILE\lab"; $ip = (Get-Content "$lab\robot_ip.txt").Trim()
function Show($ok, $item, $detail) { $s = if ($ok) { "PASS" } else { "FAIL" }; "{0,-5} {1,-18} {2}" -f $s, $item, $detail }
$ping = Test-Connection $ip -Count 2 -Quiet
Show $ping "robot ping" $ip
if (-not $ping) { "Robot does not answer: read its IP on the robot screen, then run start_gui.ps1 -RobotIp NEW_IP"; exit 1 }
ssh -o BatchMode=yes -o ConnectTimeout=5 limo "true" 2>$null
if ($LASTEXITCODE -ne 0) {
  "One-time SSH key setup: type the robot password AGX"
  Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | ssh limo "mkdir -p ~/.ssh && tr -d '\r' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
  ssh -o BatchMode=yes -o ConnectTimeout=5 limo "true" 2>$null
}
Show ($LASTEXITCODE -eq 0) "ssh key" "ssh limo works without a password"
if ($Install) {
  scp -q "$lab\install_guide.sh" limo:install_guide.sh
  if ($LASTEXITCODE -eq 0) { ssh limo "sed -i 's/\r$//' ~/install_guide.sh" }
  Show ($LASTEXITCODE -eq 0) "installer copied" "now on the robot (tmux guide): bash ~/install_guide.sh"
}
foreach ($p in 9090, 8765) {
  $ok = (Test-NetConnection $ip -Port $p -WarningAction SilentlyContinue).TcpTestSucceeded
  $d = if ($ok) { "open" } else { "closed: bridge.sh not running yet, or eduroam blocks it (4.7 SSH tunnel)" }
  Show $ok "port $p" $d
}
