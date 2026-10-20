@echo off
cd /d "%~dp0"
if not exist ".dashboard-venv\Scripts\python.exe" (
  echo Dashboard environment not found. Follow setup in BIPED_DASHBOARD.md.
  pause
  exit /b 1
)
".dashboard-venv\Scripts\python.exe" -m streamlit run app.py --server.address 127.0.0.1 --server.port 8502 --server.headless true --browser.gatherUsageStats false
