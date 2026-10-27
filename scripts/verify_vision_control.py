"""Vision actuation regression ONLY against a new PTY fake PX4; no physical camera/UART."""
from pathlib import Path
import json, os, socket, subprocess, sys, tempfile, threading, time
from verify_cpp_service import FakePX4, ServiceClient, wait
from web.server.vision_control_config import VisionControlConfig, DEFAULTS

def run():
    fake=FakePX4(); client=process=None; stop=threading.Event()
    sample=dict(source='ros2_cpp',camera_ready=True,target_found=True,camera_upright_confirmed=False,
                preview_enabled=True,tag_id=0,width=640,height=480,center=[480.,360.],measurement_age_s=.02)
    with tempfile.TemporaryDirectory(prefix='vision-flight-test-') as temp:
        folder=Path(temp)
        def feed():
            while not stop.is_set():
                value=dict(sample,written_monotonic=time.monotonic()-sample.get('test_delay',0))
                pending=folder/'status.tmp';pending.write_text(json.dumps(value));pending.replace(folder/'status.json')
                time.sleep(.03)
        worker=threading.Thread(target=feed,daemon=True);worker.start()
        try:
            with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
            fake.thread.start()
            process=subprocess.Popen([str(Path(sys.argv[1]).resolve()),'--device',fake.device,'--port',str(port),'--logs',str(folder/'logs')],
                env=dict(os.environ,DRONE_VISION_SNAPSHOT_DIR=temp,
                         DRONE_VISION_CONTROL_CONFIG=str(folder/'vision_control.json')),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            client=ServiceClient(port=port);client.start();owner='vision-fake-px4-owner'
            def state():return client.status()['data']
            def command(name,params=None):return client.request(name,params or {},owner)
            wait(lambda:state()['px4'].get('bidirectional'),8)
            assert command('CLAIM')['ok'];assert command('SET_MODE',dict(mode='OFFBOARD'))['confirmed']
            assert command('ARM')['confirmed']
            assert command('VISION_START',{})['stage']=='failed','Ground state must block vision'
            fake.xyz=[0.,0.,-1.];fake.landed=2
            wait(lambda:state()['px4'].get('landed_state')==2)
            params=dict(camera_mount_confirmed=True,vertical_enabled=True,timeout=30,lost_timeout=.3)
            assert command('MOVE',dict(kind='rotate',direction='clockwise',distance=90,speed=10,tolerance=1,timeout=30))['stage']=='running'
            assert command('STICK_START')['stage']=='failed','Task must block joystick takeover'
            assert command('VISION_START',params)['stage']=='failed','Task must block vision takeover'
            assert command('STOP_TRAJECTORY')['ok']
            stick=command('STICK_START');assert stick['confirmed']
            assert command('MOVE',dict(kind='rotate',distance=90))['stage']=='failed','Joystick must block tasks'
            assert command('VISION_START',params)['stage']=='failed','Joystick must block vision takeover'
            assert command('STICK_STOP',dict(session=stick['session']))['confirmed']
            assert command('VISION_START',params)['stage']=='running','Orientation is not a start gate'
            assert command('VISION_STOP')['confirmed']
            sample['camera_upright_confirmed']=True;time.sleep(.15)
            # Browser/JPEG are absent in this fixture. Preview advice is independently disabled.
            sample['preview_enabled']=False;time.sleep(.15)
            assert command('VISION_START',dict(params,camera_mount_confirmed=False))['stage']=='running'
            assert command('VISION_STOP')['confirmed']
            def start(options=None):
                if not command('RENEW').get('ok'):
                    assert command('CLAIM')['ok']
                result=command('VISION_START',dict(params,**(options or {})))
                assert result.get('stage')=='running',result
                wait(lambda:state()['motion'].get('id')==result['id'] and state()['motion']['state']=='running' and state()['motion'].get('phase')!='handover')
            sample['target_found']=False;sample['tag_id']=-1;sample['center']=None;time.sleep(.15)
            # Real PX4 sends EXTENDED_SYS_STATE at ~1 Hz, not the fake's default 20 Hz.
            fake.landed_interval=1.2
            start()
            wait(lambda:state()['motion'].get('phase')=='waiting_target')
            wait(lambda:state()['px4'].get('landed_age_s',0)>1,3)
            time.sleep(.15)
            assert state()['motion']['state']=='running','1 Hz landed feedback jitter must not cancel vision'
            fake.send_landed=False
            time.sleep(3.2)
            assert state()['motion']['state']=='running','Age alone must not cancel vision'
            assert command('VISION_STOP')['confirmed']
            fake.send_landed=True;fake.landed_interval=.05
            wait(lambda:state()['px4'].get('landed_age_s',99)<.2)
            start()
            assert state()['control']['setpoint']['velocity']==[0.,0.,0.]
            assert state()['motion']['state']=='running','Empty scene must not prevent enabling'
            sample['target_found']=True;sample['tag_id']=0;sample['center']=[480.,360.]
            wait(lambda:state()['motion']['setpoint']['velocity'][2]>.01 and state()['motion']['setpoint']['yaw_rate']>.01)
            point=state()['motion']['setpoint']
            assert point['position'][:2]==[0.,0.] and point['velocity'][:2]==[0.,0.]
            assert point['velocity'][2]>.05 and point['yaw_rate']>.15,'Direct proportional output, no old yaw cap'
            assert state()['control']['source']=='vision'
            # Saving the common file updates the running C++ outer loop without a GUI command.
            store=VisionControlConfig(folder/'vision_control.json')
            store.save(dict(DEFAULTS,yaw_gain=.8,lateral_gain=.4,lateral_enabled=True))
            wait(lambda:state()['motion']['params']['yaw_gain']==.8 and
                 state()['motion'].get('vision_output',{}).get('lateral_m_s',0)>.1)
            store.save(dict(DEFAULTS))
            wait(lambda:state()['motion']['params']['lateral_enabled'] is False and
                 state()['motion'].get('vision_output',{}).get('lateral_m_s',1)==0)
            for name,extra in [('STICK_START',{}),('MOVE',{'kind':'move'}),('SET_VELOCITY',{'frame':'LOCAL_NED'}),('EXECUTE_TRAJECTORY',{})]:
                assert command(name,extra)['stage']=='failed',(name,'must not mix controllers')
            assert command('VISION_STOP')['confirmed'];wait(lambda:state()['motion']['state']=='cancelled')
            sample['center']=[160.,120.];time.sleep(.12);start()
            wait(lambda:state()['motion']['setpoint']['velocity'][2]<-.01 and state()['motion']['setpoint']['yaw_rate']<-.01)
            sample['center']=[319.5,239.5]
            wait(lambda:state()['motion']['setpoint']['velocity'][2]==0 and state()['motion']['setpoint']['yaw_rate']==0)
            assert state()['motion']['setpoint']['position'][2] is not None
            sample['target_found']=False
            wait(lambda:state()['motion'].get('phase')=='waiting_target',2)
            assert state()['control']['setpoint']['velocity']==[0.,0.,0.]
            assert state()['control']['setpoint']['yaw_rate']==0
            assert state()['motion']['state']=='running'
            sample['target_found']=True;sample['center']=[480.,360.]
            wait(lambda:state()['motion'].get('phase')=='tracking' and state()['motion']['setpoint']['velocity'][2]>.01)
            assert command('VISION_STOP')['confirmed']
            start({'vertical_enabled':False})
            wait(lambda:state()['motion']['params'].get('vertical_enabled') is False and state()['motion']['setpoint']['yaw_rate']>.01)
            assert state()['motion']['setpoint']['velocity'][2]==0
            sample['test_delay']=1
            time.sleep(.5)
            assert state()['motion']['state']=='running','Snapshot age must not terminate task'
            sample['camera_ready']=False
            wait(lambda:state()['motion'].get('phase')=='waiting_target')
            assert state()['motion']['setpoint']['yaw_rate']==0
            sample['camera_ready']=True;sample['test_delay']=0
            wait(lambda:state()['motion'].get('phase')=='tracking')
            sample['center']=[float('nan'),100.]
            wait(lambda:state()['motion'].get('phase')=='waiting_target',2)
            assert state()['motion']['state']=='running'
            sample['center']=[480.,360.];time.sleep(.15)
            wait(lambda:state()['motion'].get('phase')=='tracking')
            assert command('VISION_STOP')['confirmed']
            # Legacy clients may still send these fields; neither ends a visual task.
            start({'max_vertical_excursion':.1,'timeout':5})
            initial_z=state()['px4']['local_position']['z']
            wait(lambda:state()['motion'].get('elapsed_s',0)>5.5,7)
            assert state()['motion']['state']=='running','No duration cutoff'
            assert abs(state()['px4']['local_position']['z']-initial_z)>.1,'No accumulated height cutoff'
            assert command('VISION_STOP')['confirmed']
            time.sleep(.15);start()
            fake.epoch+=1;time.sleep(.5)
            assert state()['motion']['state']=='running','EKF reset is not a vision task gate'
            wait(lambda:not state()['control']['lease_active'],22)
            assert state()['motion']['state']=='running','Vision survives browser lease expiry'
            assert command('VISION_STOP')['confirmed'],'Stop works after lease expiry'
            sample['center']=[480.,360.];sample['measurement_age_s']=2.;sample['test_delay']=2.
            result=command('VISION_START',dict(params,yaw_gain=3.,vertical_gain=1.,deadzone=.5,max_yaw_rate=.01,max_vertical_speed=.01))
            assert result['stage']=='running','Vision starts without a browser lease or age/orientation gates'
            wait(lambda:state()['motion'].get('phase')=='waiting_target')
            sample['test_delay']=0
            wait(lambda:state()['motion']['setpoint']['yaw_rate']>1 and state()['motion']['setpoint']['velocity'][2]>.4)
            point=state()['motion']['setpoint']
            assert point['yaw_rate']>1 and point['velocity'][2]>.4,'Legacy speed caps are ignored'
            sample['center']=[320.,240.]
            wait(lambda:0<state()['motion']['setpoint']['yaw_rate']<.01)
            assert state()['motion']['setpoint']['velocity'][2]>0,'No center deadzone'
            assert command('VISION_STOP')['confirmed']
            sample['measurement_age_s']=.02;sample['test_delay']=0
            # Explicit click recaptures fresh coordinates after an earlier failed takeoff/reset.
            fake.takeoff_automatically=False;fake.mode=(4,2);fake.landed=3;fake.range_cm=2
            sample['target_found']=False;sample['center']=None
            wait(lambda:state()['px4'].get('mode')=='AUTO_TAKEOFF' and state()['px4'].get('range_m')==.02)
            before=len(fake.commands)
            start()
            assert state()['px4']['mode']=='OFFBOARD','TAKEOFF state no longer requires range threshold'
            assert command('VISION_STOP')['confirmed']
            fake.mode=(4,2)
            fake.range_cm=50
            wait(lambda:state()['px4'].get('mode')=='AUTO_TAKEOFF' and state()['px4'].get('range_m')==.5)
            before=len(fake.commands);start()
            assert state()['px4']['mode']=='OFFBOARD'
            assert any(m.command==176 and int(m.param2)==6 for m in fake.commands[before:])
            wait(lambda:state()['motion'].get('phase')=='waiting_target')
            assert state()['motion']['state']=='running'
            assert command('VISION_STOP')['confirmed']
            fake.mode=(4,3);fake.landed=2
            wait(lambda:state()['px4'].get('mode')=='LOITER')
            fake.mode_result=2;before=len(fake.commands)
            assert command('VISION_START',params)['stage']=='failed'
            assert sum(m.command==176 for m in fake.commands[before:])==1,'Rejected switch must not retry'
            assert state()['px4']['mode']=='LOITER'
            fake.mode_result=0
            time.sleep(.15);start()
            before=len(fake.commands);assert command('LAND')['confirmed']
            assert any(m.command==176 and int(m.param2)==4 and int(m.param3)==6 for m in fake.commands[before:])
            assert command('VISION_START',params)['stage']=='failed'
            print('PASS: task/flight gates; no orientation/range/age/reset/lease gates; loss/data recovery; direct gains; LAND priority')
        finally:
            stop.set();worker.join(2)
            if client:client.stop()
            if process:
                process.terminate();process.wait(timeout=8)
            fake.done.set()
            if fake.thread.ident:fake.thread.join(2)
            os.close(fake.master);os.close(fake.slave)
if __name__=='__main__':run()
