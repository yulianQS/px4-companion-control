import pathlib,subprocess
for s in ['/var/log.hdd/syslog','/var/log.hdd/kern.log','/var/log.hdd/orangepi-hardware-monitor.log','/var/log/orangepi-hardware-monitor.log','/var/log.hdd/orangepi-ramlog.log']:
 p=pathlib.Path(s);print('\nLOG',s)
 if p.exists():
  lines=p.read_text(errors='replace').splitlines()
  print('LINES',len(lines))
  print('\n'.join(lines[-28:]))
for z in sorted(pathlib.Path('/sys/class/thermal').glob('thermal_zone*')):
 print('\nZONE',z.name,(z/'type').read_text().strip())
 for p in sorted(z.glob('trip_point_*_type')):print(p.name,p.read_text().strip())
 for p in sorted(z.glob('cdev*')):
  if p.is_symlink():print(p.name,str(p.resolve()))
for p in sorted(pathlib.Path('/sys/class/thermal').glob('cooling_device*')):
 print('COOLING',p.name,(p/'type').read_text().strip(),(p/'cur_state').read_text().strip(),(p/'max_state').read_text().strip())
for s in ['/sys/kernel/debug/pwm','/sys/kernel/debug/pinctrl/2000000.pinctrl/pinmux-pins']:
 p=pathlib.Path(s);print('\nDEBUG',s)
 if p.exists():
  text=p.read_text(errors='replace')
  print('\n'.join(x for x in text.splitlines() if 'PD5' in x) if 'pinctrl' in s else text)
print('\nDT PD5')
root=pathlib.Path('/proc/device-tree')
for p in root.rglob('pins'):
 try:
  if b'PD5' in p.read_bytes():
   print(p.parent)
   for n in ['pins','function','status']:
    q=p.parent/n
    if q.exists():print(n,repr(q.read_bytes()))
 except OSError:pass
for cmd in [['ps','-eo','pid,ppid,%cpu,%mem,comm','--sort=-%cpu'],['systemctl','list-timers','--all','--no-pager']]:
 r=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
 print('\nCMD',cmd,'\n','\n'.join(r.stdout.splitlines()[:22]))
