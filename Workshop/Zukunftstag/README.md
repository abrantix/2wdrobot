# Zukunftstag-Workshop: Roboter mit Claude personalisieren

Setup-Kit für einen Workshop, in dem Kinder (10–12 Jahre, ohne Programmiererfahrung) den Ax2WDRobot bauen und die Firmware mit Claude Code ("Vibe-Coding") anpassen.

Das Skript richtet einen frisch installierten Windows-11-PC ein:
VS Code, Git, Python, PlatformIO (`C:\RobotTools\venv`), Claude Code, CP210x-USB-Treiber, Klon dieser Firmware nach `C:\Robot` und ein erster Build. Danach öffnet VS Code das Projekt.

## Voraussetzungen
- Windows 11 mit `winget`, Internet
- Administratorrechte
- **Eigener Anthropic-API-Key** (optional, sonst manuelle Anmeldung in Claude Code)
- Optional: entpackter Silabs-CP210x-Treiber in `drivers\` (nicht im Repo enthalten, Lizenz: https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers)

## Benutzung
1. Diesen Ordner (`Workshop\Zukunftstag`) auf einen USB-Stick kopieren.
2. Optional `apikey.txt` daneben legen (nur der Key, eine Zeile). **Den Key nie einchecken.**
3. Optional den Treiber nach `drivers\` entpacken (`silabser.inf`).
4. Auf jedem PC `Setup.cmd` per Rechtsklick **Als Administrator ausführen** (ca. 10–20 min).
5. Desktop-Verknüpfung **Roboter** öffnen. Der grüne Knopf **ROBOTER PROGRAMMIEREN** in der Statusleiste baut und lädt hoch.

Logs landen in `logs\` neben dem Skript.

## Inhalt
- `Setup.ps1` / `Setup.cmd` – Installation
- `overlay\` – wird über das Projekt kopiert: `CLAUDE.md` (Anweisungen an Claude für Kinder), `.claude\settings.json` (Berechtigungen; **bypassPermissions**, nur für Wegwerf-PCs gedacht!), `.vscode\` (Tasks, Upload-Knopf), `tools\png_to_logo.py` (Bild → Startlogo)

## Hinweise
- Der Setup klont immer `main` von GitHub.
- Der API-Key wird lesbar in `~\.claude\settings.json` abgelegt. Prepaid-Guthaben ohne Auto-Reload verwenden und den Key nach dem Anlass löschen.
- Claude Code läuft ohne Rückfragen (YOLO). Nur auf PCs verwenden, die danach neu aufgesetzt werden.
