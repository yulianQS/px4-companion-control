#!/usr/bin/env python3
"""Record a bounded, low-load power/temperature observation in the user directory."""
import argparse,datetime,json,os,pathlib,time
parser=argparse.ArgumentParser()
parser.add_argument('--seconds',type=int,default=300)
parser.add_argument('--interval',type=float,default=10)
parser.add_argument('--output',required=True)
args=parser.parse_args()
if not 0<=args.seconds<=3600 or not 1<=args.interval<=60:
 parser.error('seconds must be 0..3600 and interval 1..60')
out=pathlib.Path(args.output).expanduser().resolve()
project=pathlib.Path('/home/pi/drone_project/logs').resolve()
if not out.is_relative_to(project):
 parser.error('output must be under /home/pi/drone_project/logs')
out.parent.mkdir(parents=True,exist_ok=True)
boot=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
deadline=time.monotonic()+args.seconds
with out.open('x',encoding='utf-8') as stream:
 while True:
  row={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'boot_id':boot,'uptime':float(pathlib.Path('/proc/uptime').read_text().split()[0]),'load':os.getloadavg(),'temperatures_c':{},'cpu_frequency_khz':{}}
  for zone in sorted(pathlib.Path('/sys/class/thermal').glob('thermal_zone*')):
   try:row['temperatures_c'][(zone/'type').read_text().strip()]=int((zone/'temp').read_text())/1000
   except (OSError,ValueError):pass
  for policy in sorted(pathlib.Path('/sys/devices/system/cpu/cpufreq').glob('policy*')):
   try:row['cpu_frequency_khz'][policy.name]=int((policy/'scaling_cur_freq').read_text())
   except (OSError,ValueError):pass
  for line in pathlib.Path('/proc/meminfo').read_text().splitlines():
   if line.startswith(('MemAvailable:','SwapFree:')):row[line.split(':')[0]]=int(line.split()[1])
  encoded=json.dumps(row,ensure_ascii=False)
  stream.write(encoded+'\n');stream.flush();os.fsync(stream.fileno())
  print(encoded,flush=True)
  remaining=deadline-time.monotonic()
  if remaining<=0:break
  time.sleep(min(args.interval,remaining))
