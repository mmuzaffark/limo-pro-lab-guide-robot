# sync_repo.ps1 [-Msg "commit message"] : robot + laptop files and logs into the GitHub repo, RESULTS.md, push
param([string]$Msg = "Real Limo Pro update $(Get-Date -Format yyyy-MM-dd)")
$lab = "$env:USERPROFILE\lab"; $R = "$env:USERPROFILE\limo_lab_repo"
if (-not (Test-Path $R)) { git clone git@github.com:mmuzaffark/limo-pro-lab-guide-robot.git $R }
New-Item -ItemType Directory -Force "$R\robot\logs", "$R\laptop" | Out-Null
scp -rq limo:guide_ws/params limo:guide_ws/scripts limo:guide_ws/maps limo:guide_ws/launch limo:guide_ws/places_lab.yaml limo:guide_env.sh limo:guide_ws/cyclone.xml "$R\robot\"
scp -q "limo:guide_ws/logs/*.csv" "$R\robot\logs\" 2>$null
scp -q "limo:guide_ws/logs/auto_map_*.txt" "$R\robot\logs\" 2>$null
Copy-Item "$lab\*.py", "$lab\*.ps1" "$R\laptop\" -Force
if (Test-Path "$lab\install_guide.sh") { Copy-Item "$lab\install_guide.sh" "$R\" -Force }
if (Test-Path "$lab\repo_root") { Copy-Item "$lab\repo_root\*" "$R\" -Recurse -Force }   # README.md, robot/patches
if (Test-Path "$lab\lab_places_map_v2.png") { Copy-Item "$lab\lab_places_map_v2.png" "$R\laptop\" -Force }
if (Test-Path "$lab\places_lab.yaml") { Copy-Item "$lab\places_lab.yaml" "$R\laptop\" -Force }
$env:PYTHONIOENCODING = "utf-8"; [Console]::OutputEncoding = [Text.Encoding]::UTF8
py -X utf8 "$lab\make_report.py" "$R\robot\logs" | Set-Content -Encoding utf8 "$R\RESULTS.md"
Set-Location $R
git add -A robot laptop RESULTS.md
foreach ($f in "install_guide.sh", "README.md") { if (Test-Path $f) { git add $f } }
git diff --cached --quiet
if ($LASTEXITCODE -eq 0) { "nothing new to commit" } else { git commit -q -m $Msg }
git push -q origin main
if ($LASTEXITCODE -eq 0) { "pushed: $(git log -1 --oneline)" }
