$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath('C:\Users\noah\Desktop\Personal Engineering\Code Projects\Negentropy')
$oldDashboard = [IO.Path]::GetFullPath((Join-Path $projectRoot 'biped_dashboard'))
if (-not (Test-Path -LiteralPath (Join-Path $oldDashboard 'app.py'))) { throw 'Dashboard source not found.' }
if (-not $oldDashboard.StartsWith($projectRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Source outside project.' }
$backupRoot = Join-Path $projectRoot ('archive\dashboard-layout-backup-' + [guid]::NewGuid().ToString('N').Substring(0,8))
$items = @(Get-ChildItem -Force -LiteralPath $oldDashboard)
$moves = foreach ($item in $items) {
    $destination = switch ($item.Name) {
        '.venv' { Join-Path $projectRoot '.dashboard-venv' }
        'tests' { Join-Path $projectRoot 'dashboard_tests' }
        'README.md' { Join-Path $projectRoot 'BIPED_DASHBOARD.md' }
        'requirements.txt' { Join-Path $projectRoot 'dashboard-requirements.txt' }
        'requirements-tested.txt' { Join-Path $projectRoot 'dashboard-requirements-tested.txt' }
        '.gitignore' { Join-Path $backupRoot 'dashboard.gitignore' }
        '__pycache__' { Join-Path $backupRoot 'old-dashboard-cache' }
        default {
            if ($item.Extension -eq '.zip') { Join-Path $backupRoot $item.Name }
            else { Join-Path $projectRoot $item.Name }
        }
    }
    $destination = [IO.Path]::GetFullPath($destination)
    if (-not $destination.StartsWith($projectRoot + '\', [StringComparison]::OrdinalIgnoreCase)) { throw "Destination outside project: $destination" }
    if (Test-Path -LiteralPath $destination) { throw "Would overwrite an existing project item: $destination" }
    [pscustomobject]@{ Source=$item.FullName; Destination=$destination }
}
if (Test-Path -LiteralPath (Join-Path $projectRoot 'run_dashboard.bat')) { throw 'Launcher already exists.' }
New-Item -Path $backupRoot -ItemType Directory | Out-Null
Copy-Item -LiteralPath (Join-Path $projectRoot 'README.md') -Destination (Join-Path $backupRoot 'root-README.md')
Copy-Item -LiteralPath (Join-Path $projectRoot '.gitignore') -Destination (Join-Path $backupRoot 'root.gitignore')
foreach ($move in $moves) { Move-Item -LiteralPath $move.Source -Destination $move.Destination }
if (@(Get-ChildItem -Force -LiteralPath $oldDashboard).Count -ne 0) { throw 'Source folder not empty; retained.' }
# Non-recursive removal of the verified, now-empty source folder only.
Remove-Item -LiteralPath $oldDashboard

$docPath = Join-Path $projectRoot 'BIPED_DASHBOARD.md'
$doc = Get-Content -Raw -LiteralPath $docPath
$doc = $doc.Replace('This dashboard is isolated in its own folder inside Negentropy.','The dashboard app and modules live directly in the Negentropy project root.')
$doc = $doc.Replace('biped_dashboard\.venv','.dashboard-venv').Replace('biped_dashboard\app.py','app.py').Replace('biped_dashboard\requirements.txt','dashboard-requirements.txt').Replace('biped_dashboard\tests','dashboard_tests')
$doc = $doc.Replace('`requirements-tested.txt`','`dashboard-requirements-tested.txt`').Replace('`requirements.txt` specifies','`dashboard-requirements.txt` specifies')
$doc = $doc.Replace('py -3.12 -m venv .dashboard-venv','py -3.12 -m venv .dashboard-venv')
$doc += "`n`nYou can also double-click ``run_dashboard.bat`` from the project root. It uses the dashboard's dedicated interpreter and runs the root ``app.py`` on port 8502. Use ``python -m pip`` and ``python -m streamlit`` through that interpreter; no environment activation is needed.`n"
Set-Content -LiteralPath $docPath -Value $doc -Encoding utf8
$appPath = Join-Path $projectRoot 'app.py'
$appText = Get-Content -Raw -LiteralPath $appPath
Set-Content -LiteralPath $appPath -Value ($appText.Replace('python -m streamlit run biped_dashboard/app.py','python -m streamlit run app.py')) -Encoding utf8
$validationPath = Join-Path $projectRoot 'VALIDATION.md'
$validation = Get-Content -Raw -LiteralPath $validationPath
Set-Content -LiteralPath $validationPath -Value ($validation.Replace('biped_dashboard\.venv','.dashboard-venv').Replace('biped_dashboard\tests','dashboard_tests')) -Encoding utf8

$readmePath = Join-Path $projectRoot 'README.md'
$readme = Get-Content -Raw -LiteralPath $readmePath
$dashboardIntro = @'

## Biped design dashboard

The dashboard lives in this main project folder. Double-click `run_dashboard.bat`,
or run this command from a PowerShell terminal in Negentropy:

```powershell
.\.dashboard-venv\Scripts\python.exe -m streamlit run app.py --server.port 8502
```

Open http://127.0.0.1:8502/. See [BIPED_DASHBOARD.md](BIPED_DASHBOARD.md) for
setup, design goals, walking preview, and SolidWorks parameter exports.
Dashboard dependencies are in `dashboard-requirements.txt`, tests in
`dashboard_tests`, and CAD exports in `cad_handoff`.

'@
$firstLineEnd = $readme.IndexOf("`n")
$readme = $readme.Substring(0,$firstLineEnd+1) + $dashboardIntro + "`n" + $readme.Substring($firstLineEnd+1)
Set-Content -LiteralPath $readmePath -Value $readme -Encoding utf8
Add-Content -LiteralPath (Join-Path $projectRoot '.gitignore') -Value "`n# Biped dashboard environment and generated test output`n.dashboard-venv/`n.pytest_cache/`ntest-results.xml" -Encoding utf8
$launcher = @'
@echo off
cd /d "%~dp0"
if not exist ".dashboard-venv\Scripts\python.exe" (
  echo Dashboard environment not found. Follow setup in BIPED_DASHBOARD.md.
  pause
  exit /b 1
)
".dashboard-venv\Scripts\python.exe" -m streamlit run app.py --server.address 127.0.0.1 --server.port 8502 --server.headless true --browser.gatherUsageStats false
'@
Set-Content -LiteralPath (Join-Path $projectRoot 'run_dashboard.bat') -Value $launcher -Encoding ascii
Write-Output "Dashboard moved to $projectRoot"
Write-Output "Previous docs, cache and backup ZIPs retained in $backupRoot"
