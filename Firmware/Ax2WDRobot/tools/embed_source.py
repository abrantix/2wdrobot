Import("env")
import os
import zipfile

# Packs the project source into embedded/source.zip. board_build.embed_files links that file into the
# firmware, and the web server hands it out as /source.zip, so the source always matches the running firmware.

project_dir = env.subst("$PROJECT_DIR")
out_dir = os.path.join(project_dir, 'embedded')
out_path = os.path.join(out_dir, 'source.zip')
root_name = os.path.basename(project_dir.rstrip('\\/'))

skip_dirs = {'.pio', '.vscode', '__pycache__', 'embedded', '.git'}
skip_files = {os.path.join('src', 'InlinePages.h')}  # generated from data/ at every build

readme = """Ax2WDRobot - Quellcode
======================

Dieser Quellcode wurde direkt aus der Firmware des Roboters heruntergeladen
(http://192.168.4.1/source.zip) und entspricht genau dem Stand, der auf dem Roboter lief.

Weiterarbeiten:
1. Visual Studio Code installieren und die Erweiterung "PlatformIO IDE" hinzufuegen
   (fuer ESP32-Boards wird die pioarduino-Plattform aus platformio.ini automatisch geladen).
2. Diesen Ordner in VS Code oeffnen.
3. Mit "Build" (Haken-Symbol) kompilieren, mit "Upload" (Pfeil) auf den Roboter flashen.

Die Datei src/InlinePages.h wird beim Build automatisch aus data/ erzeugt.
"""

os.makedirs(out_dir, exist_ok=True)

entries = []
for base, dirs, files in os.walk(project_dir):
    dirs[:] = sorted(d for d in dirs if d not in skip_dirs)
    for name in sorted(files):
        full = os.path.join(base, name)
        rel = os.path.relpath(full, project_dir)
        if rel in skip_files or name.endswith('.pyc'):
            continue
        entries.append((full, rel))

# Fixed timestamps from the files themselves keep the zip identical between builds, so nothing relinks needlessly
tmp_path = out_path + '.tmp'
with zipfile.ZipFile(tmp_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
    info = zipfile.ZipInfo(root_name + '/LIES_MICH.txt', (2000, 1, 1, 0, 0, 0))
    info.compress_type = zipfile.ZIP_DEFLATED
    zf.writestr(info, readme)
    for full, rel in entries:
        t = __import__('time').localtime(os.path.getmtime(full))
        info = zipfile.ZipInfo(root_name + '/' + rel.replace('\\', '/'), (max(t.tm_year, 1980), t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec - t.tm_sec % 2))
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = 0o644 << 16
        with open(full, 'rb') as f:
            zf.writestr(info, f.read())

old = open(out_path, 'rb').read() if os.path.exists(out_path) else None
new = open(tmp_path, 'rb').read()
if old == new:
    os.remove(tmp_path)
else:
    os.replace(tmp_path, out_path)
print("embed_source: %d files, %d bytes -> %s" % (len(entries) + 1, len(new), out_path))
