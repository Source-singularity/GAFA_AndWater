@echo off
powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process -FilePath '%~dp0zadig-2.9.exe' -Verb RunAs"
