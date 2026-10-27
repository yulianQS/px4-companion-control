"""ARM/Linux integration: real mission engine + TCP/C++ + ONLY a new PTY PX4.

No physical UART, ROS camera or real arming is used. Vision measurements are
explicit simulated inputs. The test checks protocol/feedback, not actual flight.
"""
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import threading
import time
from verify_cpp_service import FakePX4, wait
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.cpp_bridge import FlightConsole
from web.server.vision_snapshots import VisionSnapshots
from web.server.mission_client import MissionClient
from web.server.mission_catalog import BY_TYPE
from web.server.vision_control_config import VisionControlConfig, DEFAULTS


def run():
    binary=Path(sys.argv[1]).resolve()
    fake=FakePX4()
    assert fake.device.startswith('/dev/pts/')
    owner='mission-pty-test-owner'
    console=process=engine=None
    done=threading.Event()
    visible=threading.Event()
    intermittent=threading.Event();intermittent.set()
    long_gaps=threading.Event()
    centered=threading.Event()
    frozen=threading.Event()
    with tempfile.TemporaryDirectory(prefix='competition-pty-') as temp:
        root=Path(temp)
        snapshots=root/'vision';snapshots.mkdir()
        defaults=root/'defaults.json'
        VisionControlConfig(defaults).save(dict(DEFAULTS,forward_speed_m_s=4.))
        def publish():
            count=0;sample={}
            while not done.is_set():
                fake.epoch = (fake.epoch + 1) % 256
                stamp=time.monotonic()
                if not frozen.is_set():
                    sample=dict(source='ros2_cpp',written_monotonic=stamp,observation_monotonic=stamp,camera_ready=True,
                                target_found=visible.is_set() and (count%35<12 if long_gaps.is_set() else not intermittent.is_set() or count%10<5),tag_id=0,
                                center=[319.5,239.5] if centered.is_set() else [359.5,339.5],width=640,height=480,
                                measurement_age_s=.01,pose_valid=True,calibrated=True,tag_size_m=.12,distance_m=1 if centered.is_set() else 2,
                                translation_m=[.3,-.1,1 if centered.is_set() else 2])
                sample['written_monotonic']=stamp
                pending=snapshots/'status.tmp';pending.write_text(json.dumps(sample))
                pending.replace(snapshots/'status.json');done.wait(.03)
                count+=1
        publisher=threading.Thread(target=publish,daemon=True)
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
            publisher.start();fake.thread.start()
            process=subprocess.Popen([str(binary),'--device',fake.device,'--port',str(port),'--logs',str(root/'flights')],
                                     env=dict(os.environ,DRONE_VISION_SNAPSHOT_DIR=str(snapshots),DRONE_VISION_CONTROL_CONFIG=str(defaults)),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            console=FlightConsole(port=port);console.start()
            state=lambda:console.status()['data']
            wait(lambda:state()['px4'].get('bidirectional') and abs(state()['px4'].get('takeoff_speed_m_s',-1)-fake.takeoff_speed)<1e-6,8)
            assert console.client.request('CLAIM',{},owner)['ok']
            assert console.client.request('SET_MODE',{'mode':'OFFBOARD'},owner)['confirmed']
            assert console.client.request('ARM',{},owner)['confirmed']
            wait(lambda:state()['px4'].get('armed') is True)
            # Positive ground Z must not turn a relative climb into -height.
            fake.xyz[2] = .09
            wait(lambda:abs(state()['px4']['local_position']['z'] - .09) < .001)
            # This is simulated manual arming performed by the test fixture only.
            types=['takeoff','trajectory','search','recognize','lock','align','distance','land']
            blocks=[]
            for i,t in enumerate(types):
                blocks.append(dict(id='block-'+str(i),type=t,name=BY_TYPE[t]['name'],params={},branches={},timeout=30,next='block-'+str(i+1) if i+1<len(types) else None))
            blocks[0]['params']=dict(height=.2,tolerance=.02)
            blocks[1]['params']=dict(waypoints=[[0,0,-10],[10,0,-10]],fixed_yaw_deg=45,speed=.2)
            for i in (3,4,5,6):
                blocks[i]['params']=dict(settle_seconds=.1)
            blocks[3]['params']['settle_seconds']=.8
            blocks[5]['params']['coarse_settle_seconds']=.3
            blocks[6]['params']['duration']=24
            if '--merged' in sys.argv:
                blocks[2]['params']['confirm_seconds']=.8
                blocks=[b for b in blocks if b['type'] not in ('recognize','lock')]
                for a,b in zip(blocks,blocks[1:]):
                    a['next']=b['id']
            engine=MissionClient(console)
            engine.start(dict(version=1,id='pty-competition',name='PTY full competition',entry='block-0',blocks=blocks),owner)
            wait(lambda:engine.status().get('current_id')=='block-2',30)
            position=state()['px4']['local_position'].copy()
            visible.set()
            wait(lambda:any(e['event']=='search_approach_started' for e in engine.status()['events']),3)
            session=next(e['session_id'] for e in engine.status()['events'] if e['event']=='search_approach_started')
            wait(lambda:state()['motion'].get('id')==session and state()['motion'].get('params',{}).get('search_approach'),3)
            wire_start=len(fake.setpoints)
            visible.clear()
            time.sleep(.8)  # A single flash still initiates forward approach beyond 0.5s retention.
            searching=engine.status()
            assert searching['current_id']=='block-2',searching
            assert searching['blocks'].get('block-3',{}).get('status','pending')=='pending',searching
            assert state()['motion']['id']==session and state()['motion']['phase']=='search_memory_forward',state()['motion']
            velocity=state()['motion']['setpoint']['velocity']
            assert .08<math.hypot(*velocity[:2])<=.101,velocity
            point=state()['px4']['local_position']
            assert math.hypot(point['x']-position['x'],point['y']-position['y'])>.02,'Flash did not initiate approach'
            assert abs(point['z']-position['z'])<.01,'Search approach changed height'
            wait(lambda:state()['motion']['phase']=='search_reacquire',4)
            wait(lambda:math.hypot(*state()['motion']['setpoint']['velocity'][:2])<1e-6,3)
            searching=engine.status()
            assert searching['current_id']=='block-2' and state()['motion']['id']==session
            start=next(i for i,e in enumerate(searching['events']) if e['event']=='search_approach_started')
            assert not any(e.get('command') in ('MOVE','HOLD','VISION_STOP') for e in searching['events'][start:]),searching['events']
            points=[m for _,m in fake.setpoints[wire_start:]]
            assert all(math.hypot(m.vx,m.vy)<=.101 for m in points),'Search speed exceeded configured value'
            assert all(math.hypot(b.vx-a.vx,b.vy-a.vy)<.04 for a,b in zip(points,points[1:])),'Abrupt search velocity step'
            # Separate tracking epochs must never add up to a confirmed target.
            long_gaps.set();visible.set()
            time.sleep(4.2)
            assert engine.status()['current_id']=='block-2','Search accumulated evidence across expired targets'
            long_gaps.clear()
            visible.set()
            wait(lambda:engine.status().get('current_id')=='block-5',30)
            frozen.set()
            time.sleep(.3)
            assert engine.status().get('current_id')=='block-5','Repeated snapshot falsely completed alignment'
            frozen.clear()
            wait(lambda:engine.status().get('current_id')=='block-6',30)
            handoff=engine.status()
            assert any(r['event']=='coarse_alignment_handoff' and abs(r['error_px'][0])>15 and abs(r['error_px'][1])>60 for r in handoff['events']),handoff['events'][-8:]
            wait(lambda:state()['motion'].get('params',{}).get('distance_enabled'),5)
            assert state()['motion']['id']==session,'Handoff restarted vision session'
            assert not state()['motion']['params'].get('search_approach',False)
            assert state()['motion']['params']['vertical_enabled'],'Distance approach lost vertical control'
            assert state()['motion']['params']['distance_constant_speed']
            wait(lambda:state()['motion'].get('vision_output',{}).get('forward_m_s')==4.,3)
            wait(lambda:fake.setpoints and abs(math.hypot(fake.setpoints[-1][1].vx,fake.setpoints[-1][1].vy)-4.)<1e-5,3)
            intermittent.clear();centered.set()
            wait(lambda:state()['motion'].get('state')=='running' and state()['motion'].get('params',{}).get('distance_enabled'),5)
            visible.clear()
            wait(lambda:engine.status()['blocks']['block-6']['status']=='waiting',3)
            retained=engine.status()
            assert retained['retained_target']['tag_id']==0 and retained['lost_elapsed_s']>0
            visible.set()
            wait(lambda:engine.status()['blocks']['block-6']['status']=='running',3)
            assert engine.status()['lost_elapsed_s']==0
            # Disconnect the entire web/API TCP client beyond its 20 s lease.
            # C++ must keep scheduling and renew ownership itself.
            console.stop()
            time.sleep(22)
            console=FlightConsole(port=port);console.start()
            wait(lambda:state().get('service',{}).get('connected'),8)
            engine=MissionClient(console)
            wait(lambda:not engine.status()['active'],55)
            result=engine.status()
            assert result['state']=='succeeded', dict(state=result['state'],message=result['message'],current=result.get('current_name'),events=result['events'][-8:])
            log_path=binary.parent.parent/'mission_runs'/result['log_file']
            persisted=[json.loads(line) for line in log_path.read_text().splitlines()]
            assert '.partial.' not in log_path.name and not result['log_error']
            assert persisted[0]['boot_id'] and persisted[0]['utc_iso'] and persisted[0]['run_id']==result['run_id']
            assert persisted[-1]['event']=='run_finished' and persisted[-1]['state']=='succeeded'
            assert all(e.get('utc_iso') and e.get('monotonic_s') for e in persisted)
            assert all(b['status']=='succeeded' for b in result['blocks'].values()), result['blocks']
            assert any(abs(s.yaw-math.pi/4)<.03 for _,s in fake.setpoints if not s.type_mask&1024),'Fixed yaw was not forwarded to C++'
            wait(lambda:state()['px4'].get('armed') is False and state()['px4'].get('landed_state')==1,3)
            rows=result['events']
            if '--merged' not in sys.argv:
                reused=next(e for e in rows if e.get('reused_search_confirmation'))
                began=next(e for e in rows if e['event']=='block_started' and e.get('block_id')=='block-3')
                assert reused['elapsed_s']-began['elapsed_s']<.5,'Recognition repeated completed search confirmation'
            commands=[r.get('command') for r in rows if r['event']=='command']
            assert all(c in commands for c in ('TAKEOFF','EXECUTE_TRAJECTORY','VISION_START','VISION_STOP','LAND'))
            assert commands.count('VISION_START')==1,commands
            assert commands.count('VISION_STOP')==1 and commands.count('VISION_UPDATE')==2,commands
            assert any(r['event']=='coarse_alignment_handoff' for r in rows)
            # Legacy observation and coarse alignment must recover even while
            # sharing the search-approach session (the original stuck path).
            assert console.client.request('CLAIM',{},owner)['ok']
            assert console.client.request('SET_MODE',{'mode':'OFFBOARD'},owner)['confirmed']
            assert console.client.request('ARM',{},owner)['confirmed']
            wait(lambda:state()['px4'].get('armed') is True)
            intermittent.clear();centered.clear();visible.clear()
            time.sleep(.7)  # Expire the preceding run's target evidence.
            recovery=[]
            for i,t in enumerate(('takeoff','search','recognize','lock','align','distance')):
                recovery.append(dict(id='recovery-'+str(i),type=t,name=t,params={},branches={},timeout=20,
                                     next='recovery-'+str(i+1) if i<5 else None))
            recovery[0]['params']=dict(height=.15,tolerance=.02)
            recovery[2]['params']=dict(settle_seconds=.1)
            recovery[3]['params']=dict(settle_seconds=.8,lost_timeout=.3,lost_target='recovery-1')
            recovery[4]['params']=dict(coarse_horizontal_threshold=1,lost_timeout=.3,lost_target='recovery-1')
            engine.start(dict(version=1,id='pty-recovery',name='PTY loss recovery',entry='recovery-0',blocks=recovery),owner)
            wait(lambda:engine.status().get('current_id')=='recovery-1',15)
            visible.set()
            wait(lambda:engine.status().get('current_id')=='recovery-3',15)
            visible.clear()
            wait(lambda:any(e['event']=='target_lost' and e['block_id']=='recovery-3' for e in engine.status()['events']),5)
            wait(lambda:engine.status().get('current_id')=='recovery-1',3)
            assert engine.status()['active'],'Observation loss terminated recovery chain'
            visible.set()
            wait(lambda:engine.status().get('current_id')=='recovery-4',10)
            assert state()['motion']['params']['search_approach']
            visible.clear()
            wait(lambda:any(e['event']=='target_lost' and e['block_id']=='recovery-4' for e in engine.status()['events']),5)
            wait(lambda:engine.status().get('current_id')=='recovery-1',3)
            assert engine.status()['active'],'Coarse alignment loss terminated recovery chain'
            engine.stop('land')
            wait(lambda:not engine.status()['active'],5)
            wait(lambda:state()['px4'].get('armed') is False,5)
            print('PASS: PTY single-flash smooth approach, retained forward motion, long-loss local reacquisition without restart, fixed search height, confirmed seamless handoff, distance loss, LAND/API disconnect')
            print('PASS: legacy recognition reuses confirmation; lock/coarse alignment loss returns to search')
        finally:
            if console:console.stop()
            if process:
                process.terminate()
                try:out,err=process.communicate(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill();out,err=process.communicate()
                if process.returncode not in (0,-15):print('C++ test process:',out,err)
            done.set()
            if publisher.is_alive():publisher.join(2)
            fake.close()
            if hasattr(fake,'error'):raise fake.error


if __name__=='__main__':
    run()
