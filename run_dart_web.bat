@echo off
REM Launch the DART web UI (security turret).
REM Uses the project venv if present, else the system Python.
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" scripts\run_web.py
) else (
    python scripts\run_web.py
)
pause
