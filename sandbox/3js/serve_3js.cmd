@echo off
set PORT=5173
cd /d "%~dp0..\.."
start "" "http://localhost:%PORT%/sandbox/3js/skateboard_sandbox_3js.html"
python -m http.server %PORT%
