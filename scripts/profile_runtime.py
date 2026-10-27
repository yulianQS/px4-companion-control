#!/usr/bin/env python3
"""Read-only onboard profiling: htop terminal recording plus structured /proc samples."""
import argparse
import csv
import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from log_storage import boot_id, checkpoint, checkpoint_tree, session_directory, utc, write_json

ROOT = Path(__file__).resolve().parents[1]
NAMES = {'drone_service', 'web_control.py', 'camera_node', 'tag_detector_node', 'follow_preview_node'}


def text(path, default=''):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return default


def processes():
    found = {os.getpid(): 'runtime-profiler'}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            args = (path/'cmdline').read_bytes().split(b'\0')
            names = {Path(arg.decode(errors='replace')).name for arg in args if arg}
            matches = names & NAMES
            if matches:
                found[int(path.name)] = sorted(matches)[0]
        except OSError:
            pass
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=60)
    parser.add_argument('--label', default='runtime')
    parser.add_argument('--htop', action='store_true', help='Also save htop terminal output using script')
    args = parser.parse_args()
    if args.seconds < 1:
        parser.error('--seconds must be positive')
    label = ''.join(c for c in args.label if c.isalnum() or c in '-_') or 'runtime'
    output = session_directory('performance', label)
    stamp = utc()
    started = time.monotonic()
    metadata = dict(label=label, boot_id=text('/proc/sys/kernel/random/boot_id'),
                    board_clock_utc=stamp, cpuinfo=text('/proc/cpuinfo'),
                    vision_config=text(ROOT/'config/vision.yaml'), clock_ticks=os.sysconf('SC_CLK_TCK'),
                    scheduler_wait_available=Path('/proc/self/schedstat').exists())
    metadata.update(started_utc=stamp, runtime_session=os.environ.get('DRONE_LOG_SESSION',''),
                    boot_id=boot_id(), state='running')
    write_json(output/'metadata.json', metadata)
    recorder = None
    pids = processes()
    if args.htop and pids and shutil.which('htop') and shutil.which('script'):
        (output/'htoprc').write_text('hide_userland_threads=0\nshow_thread_names=1\n')
        command = 'timeout %ds htop -C -d 10 -p %s' % (args.seconds, ','.join(map(str, pids)))
        recorder = subprocess.Popen(['script', '-q', '-f', '-c', command, str(output/'htop.typescript')],
                                    env=dict(os.environ, TERM='xterm', HTOPRC=str(output/'htoprc')), stdin=subprocess.DEVNULL,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    previous = {}
    observations = previews = 0
    last_observation = last_preview = None
    ages, measurement_ages = [], []
    next_sample = started
    snapshots = Path('/dev/shm/drone_vision')
    print(output, flush=True)
    try:
        with (output/'threads.csv').open('w', newline='') as thread_file, (output/'system.csv').open('w', newline='') as system_file:
            threads = csv.writer(thread_file); system = csv.writer(system_file)
            threads.writerow(['elapsed_s', 'pid', 'tid', 'process', 'thread', 'cpu', 'cpu_percent', 'scheduler_wait_ms', 'allowed_cpus', 'utc_iso'])
            system.writerow(['elapsed_s', 'load1', 'mem_available_kb', 'temperature_c', 'cpu_frequency_khz', 'cooling_states', 'cpu_counters', 'pressure', 'utc_iso'])
            while time.monotonic()-started < args.seconds:
                now = time.monotonic()
                try:
                    snapshot = json.loads((snapshots/'status.json').read_text())
                    observation = snapshot.get('observation_monotonic')
                    if observation and observation != last_observation:
                        if last_observation is not None:
                            observations += 1
                            ages.append(max(0., now-observation)*1000)
                            age = snapshot.get('measurement_age_s')
                            if isinstance(age, (int, float)):
                                measurement_ages.append((age+max(0.,now-snapshot.get('written_monotonic',now)))*1000)
                        last_observation = observation
                    preview = (snapshots/'latest.jpg').stat().st_mtime_ns
                    if preview != last_preview:
                        previews += last_preview is not None
                        last_preview = preview
                except (OSError, ValueError):
                    pass
                if now >= next_sample:
                    next_sample = now+1
                    for pid, name in processes().items():
                        for path in (Path('/proc')/str(pid)/'task').glob('[0-9]*'):
                            try:
                                raw = (path/'stat').read_text(); fields = raw[raw.rfind(')')+2:].split()
                                ticks = int(fields[11])+int(fields[12])
                                scheduled = text(path/'schedstat').split()
                                wait_ns = int(scheduled[1]) if len(scheduled)>1 else None
                                key = (pid, int(path.name), fields[19])  # Include start time to handle TID reuse.
                                old = previous.get(key, (now, ticks, wait_ns)); dt = now-old[0]
                                previous[key] = (now, ticks, wait_ns)
                                if dt <= 0: continue
                                allowed = next((line.split(':',1)[1].strip() for line in text(path/'status').splitlines() if line.startswith('Cpus_allowed_list:')), '')
                                threads.writerow([round(now-started,3), pid, path.name, name, text(path/'comm'), fields[36],
                                                  round((ticks-old[1])/os.sysconf('SC_CLK_TCK')/dt*100,2),
                                                  round((wait_ns-old[2])/1e6,3) if wait_ns is not None and old[2] is not None else '', allowed, utc()])
                            except (OSError, ValueError, IndexError):
                                pass
                    temps = {text(z/'type'): int(text(z/'temp','0'))/1000 for z in Path('/sys/class/thermal').glob('thermal_zone*')}
                    freqs = {p.name: text(p/'scaling_cur_freq') for p in Path('/sys/devices/system/cpu/cpufreq').glob('policy*')}
                    cooling = {text(p/'type'):text(p/'cur_state') for p in Path('/sys/class/thermal').glob('cooling_device*')}
                    available = next((s.split()[1] for s in text('/proc/meminfo').splitlines() if s.startswith('MemAvailable:')), '')
                    counters = [s for s in text('/proc/stat').splitlines() if s.startswith('cpu')]
                    pressure = {p.name:text(p) for p in Path('/proc/pressure').glob('*')}
                    system.writerow([round(now-started,3), os.getloadavg()[0], available, json.dumps(temps), json.dumps(freqs), json.dumps(cooling), json.dumps(counters), json.dumps(pressure), utc()])
                    checkpoint(thread_file);checkpoint(system_file)
                    if recorder and (output/'htop.typescript').exists():
                        with (output/'htop.typescript').open('rb') as recording:
                            os.fsync(recording.fileno())
                time.sleep(.02)
            checkpoint(thread_file);checkpoint(system_file)
    except BaseException as error:
        metadata.update(ended_utc=utc(), state='interrupted', error=str(error))
        write_json(output/'metadata.json', metadata)
        raise
    finally:
        if recorder:
            try: recorder.wait(timeout=3)
            except subprocess.TimeoutExpired: recorder.terminate();recorder.wait(timeout=3)
        checkpoint_tree(output, {})
    elapsed = time.monotonic()-started
    summary = dict(seconds=elapsed, observation_updates_per_s=observations/elapsed,
                   jpeg_updates_per_s=previews/elapsed, observation_age_mean_ms=sum(ages)/len(ages) if ages else None,
                   observation_age_max_ms=max(ages,default=None),
                   measurement_age_mean_ms=sum(measurement_ages)/len(measurement_ages) if measurement_ages else None,
                   measurement_age_max_ms=max(measurement_ages,default=None),
                   note='Observation age is time since result reception; measurement age includes image processing. Update counts are not camera sensor FPS or flight validation.')
    summary.update(ended_utc=utc(), boot_id=boot_id())
    write_json(output/'summary.json', summary)
    metadata.update(ended_utc=utc(), state='completed')
    write_json(output/'metadata.json', metadata)
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
