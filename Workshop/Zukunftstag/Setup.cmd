@echo off
rem Startet Setup.ps1 (als Administrator ausfuehren)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0Setup.ps1"
pause
