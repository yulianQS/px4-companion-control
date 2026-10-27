"""Relative-ground takeoff on a new PTY. Never opens physical UART/USB."""
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from verify_cpp_service import FakePX4, ServiceClient, wait


def run():
    binary = Path(sys.argv[1]).resolve()
    fake = FakePX4()
    assert fake.device.startswith('/dev/pts/')
    client = process = None
    with tempfile.TemporaryDirectory(prefix='relative-takeoff-') as temp:
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0));port = probe.getsockname()[1]
            fake.thread.start()
            process = subprocess.Popen([str(binary), '--device', fake.device, '--port', str(port),
                                        '--logs', temp], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            client = ServiceClient(port=port);client.start()
            owner = 'relative-takeoff-test-owner'
            def state():return client.status()['data']
            def command(name, params=None):return client.request(name, params or {}, owner)
            def ground(z):
                fake.armed=False;fake.landed=1;fake.xyz=[0.,0.,z];fake.mode=(4,3)
                wait(lambda:state()['px4'].get('armed') is False and state()['px4'].get('mode')=='LOITER'
                     and abs(state()['px4'].get('local_position',{}).get('z',1e6)-z)<.001)
                assert command('CLAIM')['ok']
                assert command('SET_MODE',{'mode':'OFFBOARD'})['confirmed']
            wait(lambda:state()['px4'].get('bidirectional') and 'takeoff_speed_m_s' in state()['px4'],8)
            ground(0.)
            before=len(fake.commands)
            assert command('TAKEOFF',{'height':.5})['stage']=='failed'
            assert len(fake.commands)==before,'Takeoff must not automatically arm'
            # Reproduce both recorded ground offsets. Deliberately publish a
            # contradictory native target to prove it cannot replace our goal.
            fake.send_native_target=True;fake.native_target_override=-.5
            for z,height in [(0.,.5),(.93,.5),(.70,.5),(-.4,.1)]:
                ground(z)
                assert command('ARM')['confirmed']
                wait(lambda:state()['px4'].get('armed') is True)
                before=len(fake.commands);first=len(fake.setpoints)
                result=command('TAKEOFF',{'height':height,'tolerance':.02})
                assert result['stage']=='running',result
                wait(lambda:state()['motion'].get('id')==result['id'] and state()['motion']['state']=='succeeded',10)
                motion=state()['motion'];expected=z-height
                assert abs(motion['ground_z_m']-z)<.001
                assert abs(motion['goal_position'][2]-expected)<.001
                assert abs(fake.xyz[2]-expected)<.001,(z,height,fake.xyz)
                assert abs(state()['control']['height_above_takeoff_ground_m']-height)<.001
                points=[m for _,m in fake.setpoints[first:] if not m.type_mask&4]
                assert points and all(expected-.001<=m.z<=z+.001 for m in points)
                assert fake.mode==(6,0) and state()['control']['zero_stream']
                assert not fake.commands[before:] and not fake.param_sets,'No NAV_TAKEOFF, mode/arm or parameter writes'
            # A stationary aircraft cannot be marked complete merely because
            # the reference reached its endpoint. Timeout keeps actual position.
            ground(0.);assert command('ARM')['confirmed']
            first=len(fake.setpoints)
            result=command('TAKEOFF',dict(height=.5,speed=2,acceleration=0))
            assert result['stage']=='running',result
            wait(lambda:state()['motion'].get('id')==result['id'] and state()['motion']['state']=='succeeded',5)
            assert max(abs(m.vz) for _,m in fake.setpoints[first:])>=1.99
            ground(.93);assert command('ARM')['confirmed']
            fake.follow_setpoints=False
            result=command('TAKEOFF',{'height':.5,'timeout':2})
            assert result['stage']=='running'
            wait(lambda:state()['motion'].get('id')==result['id'] and state()['motion']['state']=='failed',4)
            assert abs(fake.xyz[2]-.93)<.001
            assert abs(state()['control']['setpoint']['position'][2]-.93)<.001
            # Existing explicit LAND must still preempt an active takeoff.
            fake.follow_setpoints=True
            ground(.7);assert command('ARM')['confirmed']
            assert command('TAKEOFF',{'height':.5})['stage']=='running'
            assert command('LAND').get('accepted')
            wait(lambda:state()['px4'].get('armed') is False)
            assert fake.mode==(4,6)
            print('PASS: PTY relative takeoff at ground Z 0/+0.93/+0.70/-0.40; bounded wire targets, real feedback completion, no native altitude/parameter writes, timeout and LAND preemption')
        finally:
            if client:client.stop()
            if process:
                process.terminate();out,err=process.communicate(timeout=8)
                assert process.returncode==0,(out,err)
            fake.close()
            assert not hasattr(fake,'error'),getattr(fake,'error',None)


if __name__=='__main__':run()
