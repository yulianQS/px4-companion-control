import os,pathlib,subprocess
paths=['/var/log.hdd','/etc/default/orangepi-ramlog','/etc/default/armbian-ramlog','/etc/default/log2ram','/etc/systemd/journald.conf','/var/log/syslog','/var/log/kern.log']
for s in paths:
 p=pathlib.Path(s)
 print('\nPATH',s)
 if p.is_dir():
  for q in sorted(p.iterdir()): print(q.name,q.stat().st_size)
 elif p.exists():
  if 'syslog' in s or 'kern.log' in s:
   lines=p.read_text(errors='replace').splitlines()
   print('FIRST', '\n'.join(lines[:6]))
   hits=[x for x in lines if any(v in x.lower() for v in ['panic','oom-kill','out of memory','critical temperature','power key','power button','powering off','system is powering','shutdown','reboot:','reset reason','reset cause','watchdog','under-voltage','undervoltage','voltage low','bootcount'])]
   print('SIGNALS','\n'.join(hits[-45:]))
  else: print(p.read_text(errors='replace'))
 else: print('absent')
for name in sorted(pathlib.Path('/sys/class/power_supply').iterdir()):
 print('\nPOWER',name)
 for fn in ['uevent','online','voltage_now','voltage_max','current_max','usb_type']:
  p=name/fn
  if p.exists():
   try: print(fn,p.read_text().strip())
   except OSError as e: print(e)
for zone in sorted(pathlib.Path('/sys/class/thermal').glob('thermal_zone*')):
 print('THERMAL',zone.name,(zone/'type').read_text().strip(),(zone/'temp').read_text().strip())
 for p in sorted(zone.glob('trip_point_*_temp')):
  print(p.name,p.read_text().strip())
