$port = 5173
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectRoot = Split-Path -Parent (Split-Path -Parent $scriptDir)

Set-Location $projectRoot

Start-Process "http://localhost:$port/sandbox/3js/skateboard_sandbox_3js.html"
python -m http.server $port
