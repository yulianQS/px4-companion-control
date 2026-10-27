"""Distance-control integration against PTY fake PX4 only, never real hardware."""
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
from verify_cpp_service import FakePX4, ServiceClient, wait
from web.server.vision_control_config import VisionControlConfig, DEFAULTS

def run():
    fake=FakePX4(); fake.armed=True; fake.mode=(6,0); fake.landed=2; fake.xyz=[0.,0.,-1.]
    sample=dict(source='ros2_cpp',camera_ready=True,target_found=True,tag_id=0,width=640,height=480,
        center=[319.5,239.5],measurement_age_s=.01,calibrated=True,pose_valid=True,distance_m=2.,tag_size_m=.12)
    stop=threading.Event(); process=client=None
    with tempfile.TemporaryDirectory(prefix='distance-pty-') as temp:
        folder=Path(temp)
        defaults_path=folder/'vision_control.json'
        VisionControlConfig(defaults_path).save(dict(DEFAULTS,yaw_gain=.8,distance_enabled=False))
        def feed():
            while not stop.is_set():
                pending=folder/'status.tmp'
                pending.write_text(json.dumps(dict(sample,written_monotonic=time.monotonic())))
                pending.replace(folder/'status.json');stop.wait(.02)
        worker=threading.Thread(target=feed,daemon=True);worker.start()
        try:
            with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
            fake.thread.start()
            process=subprocess.Popen([str(Path(sys.argv[1]).resolve()),'--device',fake.device,'--port',str(port),
                '--logs',str(folder/'logs')],env=dict(os.environ,DRONE_VISION_SNAPSHOT_DIR=temp,
                DRONE_VISION_CONTROL_CONFIG=str(defaults_path)),
                stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            client=ServiceClient(port=port);client.start()
            def state():return client.status()['data']
            def command(name,p=None):return client.request(name,p or {},'distance-test-owner')
            def output():return state()['motion'].get('vision_output',{})
            wait(lambda:state()['px4'].get('bidirectional'),8)
            wait(lambda:state()['px4'].get('armed') is True and state()['px4'].get('landed_state')==2 and state()['px4'].get('mode')=='OFFBOARD')
            assert not fake.setpoints,'Monitor-only startup'
            first=command('VISION_START');assert first['stage']=='running',first
            wait(lambda:state()['motion']['params'].get('yaw_gain')==.8)
            assert state()['motion']['params']['distance_enabled'] is False
            assert command('VISION_STOP')['confirmed']
            # Constant-speed distance blocks must send the requested velocity,
            # independent of PID gains and an inherited search acceleration ramp.
            sample.update(target_found=True,pose_valid=True,distance_m=1.4,tag_size_m=.12,center=[319.5,239.5])
            fake.heading=0
            wait(lambda:abs(state()['px4']['attitude_deg'][2])<.1)
            started=command('VISION_START',dict(distance_enabled=True,distance_constant_speed=True,
                distance_tolerance_m=.1,tag_size_m=.12,target_distance_m=1.,
                distance_gain=0.,distance_ki=0.,distance_kd=0.,forward_speed_m_s=4.,
                search_approach=False,search_acceleration=.2))
            assert started['stage']=='running',started
            wait(lambda:output().get('forward_m_s')==4.)
            wait(lambda:fake.setpoints and abs(fake.setpoints[-1][1].vx-4.)<1e-6)
            sample['distance_m']=1.15
            wait(lambda:abs(state()['motion']['vision_measurement'].get('distance_m',0)-1.15)<1e-6)
            assert output()['forward_m_s']==4.,'Small distance error must not reduce cruise speed'
            sample['distance_m']=1.05
            wait(lambda:output().get('forward_m_s')==0.)
            assert state()['motion']['setpoint']['velocity'][:2]==[0.,0.]
            sample['distance_m']=.7
            wait(lambda:output().get('forward_m_s')==-4.)
            wait(lambda:fake.setpoints and abs(fake.setpoints[-1][1].vx+4.)<1e-6)
            assert command('VISION_UPDATE',dict(forward_speed_m_s=0.,distance_gain=1.))['confirmed']
            wait(lambda:abs(output().get('forward_m_s',0)+.3)<.02)
            assert state()['motion']['forward_control_mode']=='distance_pid'
            assert command('VISION_STOP')['confirmed']
            sample['distance_m']=2.
            started=command('VISION_START',dict(distance_enabled=True,vertical_enabled=True,
                tag_size_m=.12,target_distance_m=1.,distance_gain=.3,forward_speed_m_s=.2))
            assert started.get('stage')=='running',started
            wait(lambda:abs(output().get('forward_m_s',0)-.2)<1e-6)
            point=state()['motion']['setpoint']
            assert point['position'][:2]==[None,None] and point['velocity'][0]>.19
            wait(lambda:fake.setpoints and fake.setpoints[-1][1].vx>.19)
            wire=fake.setpoints[-1][1]
            assert wire.type_mask&3==3 and wire.type_mask&24==0,'XY velocity active, XY position ignored'
            assert command('VISION_UPDATE',dict(target_distance_m=2.5))['confirmed'],'Update requires no browser lease'
            wait(lambda:abs(output().get('forward_m_s',0)+.15)<1e-6)
            assert command('VISION_UPDATE',dict(forward_speed_m_s=.05))['confirmed']
            wait(lambda:abs(output().get('forward_m_s',0)+.05)<1e-6)
            assert command('VISION_UPDATE',dict(distance_gain=1.,forward_speed_m_s=0.))['confirmed']
            wait(lambda:abs(output().get('forward_m_s',0)+.5)<1e-6)
            assert command('VISION_UPDATE',dict(tag_size_m=.24))['confirmed']
            wait(lambda:abs(output().get('forward_m_s',0)-1.5)<1e-6)
            # User policy: loss preserves the last forward NED vector for 3s,
            # with one task ID and no new Offboard handshake. Recovery resumes.
            old_id=state()['motion']['id'];before_point=state()['motion']['setpoint']
            sample['target_found']=False
            wait(lambda:state()['motion'].get('phase')=='lost_target_forward')
            continued=state()['motion']['setpoint']
            assert continued['velocity'][:2]==before_point['velocity'][:2]
            assert continued['velocity'][2]==0 and continued['yaw_rate']==0
            time.sleep(.35)
            assert state()['motion']['id']==old_id and output()['forward_m_s']>1
            sample['target_found']=True
            wait(lambda:state()['motion'].get('phase')=='tracking')
            assert state()['motion']['id']==old_id
            sample['target_found']=False
            began=time.monotonic()
            wait(lambda:state()['motion'].get('phase')=='lost_target_forward')
            wait(lambda:state()['motion'].get('phase')=='waiting_target',4)
            assert time.monotonic()-began>=2.8
            assert state()['motion']['id']==old_id and output()['forward_m_s']==0
            sample['target_found']=True
            # Missing metric pose must not disable the existing pixel yaw/vertical axes.
            sample['pose_valid']=False;sample['center']=[480.,360.]
            wait(lambda:output().get('forward_m_s')==0 and output().get('yaw_rad_s',0)>.1)
            wait(lambda:output().get('vertical_m_s',0)>.05)
            wait(lambda:fake.setpoints and fake.setpoints[-1][1].vz>.05)
            sample['center']=[480.,120.]
            wait(lambda:output().get('vertical_m_s',0)<-.05)
            wait(lambda:fake.setpoints and fake.setpoints[-1][1].vz<-.05)
            assert state()['motion']['setpoint']['position'][0] is not None
            sample['target_found']=False
            wait(lambda:state()['motion'].get('phase')=='waiting_target')
            assert all(v==0 for v in output().values())
            sample.update(target_found=True,pose_valid=True,center=[319.5,239.5])
            wait(lambda:output().get('forward_m_s',0)>1)
            sample['target_found']=False
            wait(lambda:state()['motion'].get('phase')=='lost_target_forward')
            assert command('VISION_STOP')['confirmed']
            wait(lambda:state()['motion']['state']=='cancelled')
            assert state()['control']['setpoint']['velocity']==[0.,0.,0.], 'Explicit stop preempts loss continuation'
            sample['target_found']=True
            fake.mode=(4,3)
            wait(lambda:state()['px4'].get('mode')=='LOITER')
            fake.heading=math.pi/2
            wait(lambda:abs(state()['px4']['attitude_deg'][2]-90)<.1)
            assert command('VISION_START',dict(distance_enabled=True,target_distance_m=1.,distance_gain=.3))['stage']=='running'
            wait(lambda:output().get('forward_m_s',0)>.2)
            point=state()['motion']['setpoint']
            assert abs(point['velocity'][0])<.001 and point['velocity'][1]>.29,'Body forward rotates to NED Y at 90deg'
            assert command('VISION_UPDATE',dict(distance_enabled=False))['confirmed']
            wait(lambda:output().get('forward_m_s')==0)
            assert state()['motion']['setpoint']['position'][0] is not None
            assert command('VISION_STOP')['confirmed']
            print('PASS: PTY-only calibrated distance, forward/reverse, NED rotation, wire mask, live PID/speed/size updates, loss/recovery')
        finally:
            stop.set();worker.join(2)
            if client:client.stop()
            if process:process.terminate();process.wait(timeout=8)
            fake.done.set()
            if fake.thread.ident:fake.thread.join(2)
            os.close(fake.master);os.close(fake.slave)

if __name__=='__main__':run()
