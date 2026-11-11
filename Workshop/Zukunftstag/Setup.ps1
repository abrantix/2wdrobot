# Zukunftstag Setup - als Administrator ausfuehren (Rechtsklick auf Setup.cmd -> Als Administrator).
# Voraussetzungen: Internet (GitHub-Clone), Windows 11 mit winget. apikey.txt neben diesem Skript ist optional.
$ErrorActionPreference = "Stop"
$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) { throw "Bitte als Administrator starten (Rechtsklick auf Setup.cmd -> Als Administrator ausfuehren)." }
$Stick   = $PSScriptRoot
$logDir = Join-Path $Stick "logs"
try { New-Item -ItemType Directory -Path $logDir -Force | Out-Null } catch { $logDir = $env:TEMP }
$logFile = Join-Path $logDir ("setup_{0}_{1:yyyyMMdd_HHmmss}.log" -f $env:COMPUTERNAME, (Get-Date))
Start-Transcript -Path $logFile | Out-Null
Write-Host "Log: $logFile"
Write-Host ("OS: " + (Get-CimInstance Win32_OperatingSystem).Caption + " " + (Get-CimInstance Win32_OperatingSystem).Version + ", winget: " + ((winget --version 2>&1) -join " "))
$Target  = "C:\Robot"
$ToolDir = "C:\RobotTools"
$Venv    = "$ToolDir\venv"
# Firmware kommt von GitHub; $Ref am besten ein Tag (z.B. "zukunftstag") statt "main", damit spaetere Aenderungen nichts kaputtmachen.
$Repo    = "https://github.com/abrantix/2wdrobot"
$Ref     = "main"

function Step($t) { Write-Host "`n=== $t ===" -ForegroundColor Cyan }
function Refresh-Path {
    $env:Path = [Environment]::GetEnvironmentVariable("Path", "Machine") + ";" + [Environment]::GetEnvironmentVariable("Path", "User")
}
function Winget-Install($id) {
    winget install --id $id -e --silent --accept-package-agreements --accept-source-agreements
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne -1978335189) { Write-Warning "winget $id Exitcode $LASTEXITCODE" }
}

$keyFile = Join-Path $Stick "apikey.txt"
$apiKey = if (Test-Path $keyFile) { (Get-Content $keyFile -Raw).Trim() } else { "" }

Step "USB-Treiber (optional)"
if (Test-Path "$Stick\drivers") {
    Get-ChildItem "$Stick\drivers" -Filter *.inf -Recurse | ForEach-Object { pnputil /add-driver $_.FullName /install | Out-Host }
} else { Write-Warning "Kein drivers-Ordner auf dem Stick." }

Step "Programme installieren (VS Code, Git, Python)"
Winget-Install "Microsoft.VisualStudioCode"
Winget-Install "Git.Git"
Winget-Install "Python.Python.3.12"
Refresh-Path

Step "Claude Code installieren"
winget install --id Anthropic.ClaudeCode -e --silent --accept-package-agreements --accept-source-agreements
if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne -1978335189) {
    Write-Warning "winget-Paket nicht verfuegbar, nutze offiziellen Installer"
    Invoke-RestMethod https://claude.ai/install.ps1 | Invoke-Expression
}
Refresh-Path

Step "Lange Pfade in Windows aktivieren (ESP32-Bibliotheken haben sehr lange Pfade)"
Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name LongPathsEnabled -Value 1 -Type DWord
git config --system core.longpaths true

Step "PlatformIO und Pillow in eigener Umgebung"
$py = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $py) { $py = (Get-ChildItem "$env:ProgramFiles\Python312\python.exe", "$env:LOCALAPPDATA\Programs\Python\Python312\python.exe" -ErrorAction SilentlyContinue | Select-Object -First 1).FullName }
New-Item -ItemType Directory -Path $ToolDir -Force | Out-Null
if (-not (Test-Path "$Venv\Scripts\python.exe")) { & $py -m venv $Venv }
& "$Venv\Scripts\python.exe" -m pip install --upgrade pip platformio pillow
$userPath = [Environment]::GetEnvironmentVariable("Path", "User")
if ($userPath -notlike "*$Venv\Scripts*") {
    [Environment]::SetEnvironmentVariable("Path", "$Venv\Scripts;$userPath", "User")
}
# "python" soll in der Umgebung Pillow finden
$env:Path = "$Venv\Scripts;$env:Path"

Step "VS Code Erweiterungen"
$code = "$env:LOCALAPPDATA\Programs\Microsoft VS Code\bin\code.cmd"
if (-not (Test-Path $code)) { $code = "$env:ProgramFiles\Microsoft VS Code\bin\code.cmd" }
foreach ($ext in "pioarduino.pioarduino-ide", "platformio.platformio-ide") {
    # Wir bauen mit dem PlatformIO aus $Venv (gruener Knopf); die IDE-Erweiterungen bringen ein eigenes,
    # sich selbst aktualisierendes Core mit (pioarduino 6.2.0.post1 lehnte z.B. Plattform 55.03.312 ab)
    $ErrorActionPreference = "Continue"
    & $code --uninstall-extension $ext 2>&1 | ForEach-Object { "$_" } | Out-Host
    $ErrorActionPreference = "Stop"
}
foreach ($ext in "anthropic.claude-code", "ms-vscode.cpptools", "seunlanlege.action-buttons") {
    & $code --install-extension $ext --force
}

Step "Projekt nach $Target kopieren"
if (Test-Path $Target) { Remove-Item $Target -Recurse -Force }
$clone = Join-Path $env:TEMP ("robot_clone_" + [guid]::NewGuid().ToString("N"))
$ErrorActionPreference = "Continue"   # git schreibt Fortschritt nach stderr
$cloneOut = & git clone --quiet --depth 1 --branch $Ref $Repo $clone 2>&1 | ForEach-Object { "$_" }
$cloneExit = $LASTEXITCODE
$cloneOut | Out-Host
$ErrorActionPreference = "Stop"
if ($cloneExit -ne 0 -or -not (Test-Path "$clone\Firmware\Ax2WDRobot\platformio.ini")) {
    throw "GitHub-Clone von $Repo ($Ref) fehlgeschlagen - Internet pruefen."
}
Write-Host "Firmware von GitHub ($Ref) geladen."
robocopy "$clone\Firmware\Ax2WDRobot" $Target /E /XD .git | Out-Null
Copy-Item "$Stick\overlay\*" $Target -Recurse -Force
New-Item -ItemType Directory -Path "$Target\bilder" -Force | Out-Null
Remove-Item $clone -Recurse -Force -ErrorAction SilentlyContinue
git config --global --get user.name  | Out-Null; if ($LASTEXITCODE -ne 0) { git config --global user.name "Zukunftstag" }
git config --global --get user.email | Out-Null; if ($LASTEXITCODE -ne 0) { git config --global user.email "zukunftstag@example.com" }
git config --global --add safe.directory ($Target -replace "\\", "/")
Push-Location $Target
git init -q
git add -A
git commit -q -m "Original"
Pop-Location

Step "Claude: API-Key und Modell (Benutzer-Einstellungen)"
if ($apiKey) {
    $claudeDir = "$env:USERPROFILE\.claude"
    New-Item -ItemType Directory -Path $claudeDir -Force | Out-Null
    $settings = @{
        skipDangerousModePermissionPrompt = $true
        env = @{
            ANTHROPIC_API_KEY = $apiKey
            ANTHROPIC_MODEL   = "sonnet"
            CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC = "1"
        }
    } | ConvertTo-Json -Depth 5
    [IO.File]::WriteAllText("$claudeDir\settings.json", $settings, (New-Object Text.UTF8Encoding $false))

    $cfgPath = "$env:USERPROFILE\.claude.json"
    $cfg = if (Test-Path $cfgPath) { Get-Content $cfgPath -Raw | ConvertFrom-Json } else { [pscustomobject]@{} }
    $cfg | Add-Member -NotePropertyName hasCompletedOnboarding -NotePropertyValue $true -Force
    $cfg | Add-Member -NotePropertyName customApiKeyResponses -NotePropertyValue ([pscustomobject]@{
        approved = @($apiKey.Substring([Math]::Max(0, $apiKey.Length - 20)))
        rejected = @()
    }) -Force
    [IO.File]::WriteAllText($cfgPath, ($cfg | ConvertTo-Json -Depth 10), (New-Object Text.UTF8Encoding $false))
} else {
    Write-Warning "Kein apikey.txt gefunden - Claude-Konfiguration uebersprungen (Anmeldung dann manuell)."
}

Step "Ersten Build ausfuehren (laedt ESP32-Toolchain, dauert einige Minuten)"
Push-Location $Target
& "$Venv\Scripts\pio.exe" run
$buildOk = ($LASTEXITCODE -eq 0)
Pop-Location

Step "Desktop-Verknuepfung"
$lnk = (New-Object -ComObject WScript.Shell).CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\Roboter.lnk")
$lnk.TargetPath = $code
$lnk.Arguments  = "`"$Target`""
$lnk.Save()
$lnkPath = $lnk.FullName

Step "VS Code mit Projekt oeffnen"
# explorer.exe startet die Verknuepfung ohne Administratorrechte (sonst laeuft VS Code "elevated")
Start-Process explorer.exe $lnkPath

if ($buildOk) { Write-Host "`nFERTIG - Build erfolgreich." -ForegroundColor Green }
else { Write-Host "`nSetup fertig, aber der Build ist fehlgeschlagen (Internet? siehe Meldungen oben)." -ForegroundColor Red }
Stop-Transcript | Out-Null
