#!/usr/bin/python3
"""One-time, guarded UART7 configuration. Run with sudo; reboot separately."""
from pathlib import Path
from datetime import datetime, timezone
import grp
import os
import pwd
import shutil
import subprocess

root = Path('/home/pi/drone_project')
config = Path('/boot/orangepiEnv.txt')
overlay = Path('/boot/dtb/allwinner/overlay/sun60i-a733-uart7.dtbo')
base = Path('/boot/dtb/allwinner/sun60i-a733-orangepi-zero3w.dtb')
if os.geteuid() != 0:
    raise SystemExit('Run with sudo')
before = config.read_text()
if 'fdtfile=allwinner/sun60i-a733-orangepi-zero3w.dtb' not in before or 'overlay_prefix=sun60i-a733' not in before:
    raise SystemExit('Unexpected board configuration; no change')
if not overlay.is_file() or not base.is_file():
    raise SystemExit('Required vendor device-tree files missing; no change')
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
backups = root / 'config/backups'
backups.mkdir(parents=True, exist_ok=True)
trial = backups / ('uart7-validated-' + stamp + '.dtb')
subprocess.run(['fdtoverlay', '-i', str(base), '-o', str(trial), str(overlay)], check=True)
status = subprocess.check_output(['fdtget', str(trial), '/soc@3000000/uart@7080000', 'status'], text=True).strip()
if status != 'okay':
    raise SystemExit('Offline overlay validation failed; no boot change')
lines = before.splitlines()
indexes = [i for i, line in enumerate(lines) if line.startswith('overlays=')]
if len(indexes) > 1:
    raise SystemExit('Multiple overlays assignments; no change')
if indexes:
    i = indexes[0]
    items = lines[i].split('=', 1)[1].split()
    if 'uart7' not in items:
        items.append('uart7')
        lines[i] = 'overlays=' + ' '.join(items)
else:
    lines.append('overlays=uart7')
after = '\n'.join(lines) + '\n'
with (root / 'logs/configuration_changes.txt').open('a') as log:
    log.write('\n' + stamp + '\n')
    if before != after:
        backup = backups / ('orangepiEnv.txt.' + stamp)
        with backup.open('x') as file:
            file.write(before)
        shutil.copystat(config, backup)
        if config.read_text() != before:
            raise SystemExit('Configuration changed concurrently; aborting')
        config.write_text(after)
        assert config.read_text() == after
        log.write('Added overlays=uart7 using vendor DTBO. Backup: ' + str(backup) + '\n')
        print('BOOT_CONFIG_UPDATED', backup)
    else:
        log.write('UART7 already configured; no boot change.\n')
    user = pwd.getpwnam('pi')
    groups = os.getgrouplist('pi', user.pw_gid)
    if grp.getgrnam('dialout').gr_gid not in groups:
        subprocess.run(['usermod', '-aG', 'dialout', 'pi'], check=True)
        log.write('Added pi to dialout; effective after new login.\n')
        print('DIALOUT_ADDED')
    else:
        log.write('pi already in dialout.\n')
print('Offline DT overlay verified; reboot required if newly enabled.')
