"""Forced-disarm regression against a new PTY only; never physical hardware."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from verify_cpp_service import FakePX4, wait
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.cpp_bridge import ServiceClient


def run():
    binary=Path(sys.argv[1]).resolve()
    fake=FakePX4()
    client=process=None
    with tempfile.TemporaryDirectory(prefix='emergency-pty-') as directory:
        root=Path(directory)
        (root/'config').mkdir()
        catalogue=Path(__file__).resolve().parents[1]/'config/mission_blocks.json'
        (root/'config/mission_blocks.json').write_bytes(catalogue.read_bytes())
        try:
            with socket.socket() as probe:
                probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
            fake.thread.start()
            process=subprocess.Popen([str(binary),'--device',fake.device,'--port',str(port),'--logs',str(root/'flights')],
                                     env=dict(os.environ,DRONE_PROJECT_ROOT=str(root)),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            client=ServiceClient(port=port);client.start()
            state=lambda:client.status()['data']
            wait(lambda:state()['px4'].get('bidirectional'),8)
            owner='emergency-test-owner'
            request=lambda name,p={}:client.request(name,p,owner)
            assert request('CLAIM')['ok']
            assert request('SET_MODE',{'mode':'OFFBOARD'})['confirmed']
            assert request('ARM')['confirmed']
            # Actual landing telemetry says airborne; ordinary disarm must refuse.
            fake.xyz[2]=-.5;fake.landed=2;fake.follow_setpoints=False
            wait(lambda:state()['px4'].get('landed_state')==2)
            assert request('DISARM')['stage']=='failed'
            document=dict(version=1,id='emergency-test',name='PTY hover',entry='hover',blocks=[
                dict(id='hover',type='hover',name='hover',params=dict(height=.5,duration=60),next=None,branches={},timeout=90)])
            assert request('MISSION_START',document)['ok']
            wait(lambda:state()['mission'].get('active'))
            result=client.request('EMERGENCY_STOP',{},'different-client-no-lease')
            assert result['confirmed'],result
            wait(lambda:not state()['mission'].get('active'))
            wait(lambda:state()['px4'].get('armed') is False)
            assert any(c.command==400 and c.param1==0 and c.param2==21196 for c in fake.commands)
            count=len(fake.commands);time.sleep(.3)
            assert not any(c.command in (22,176) or (c.command==400 and c.param1==1) for c in fake.commands[count:])
            assert not state()['control']['zero_stream']
            points=len(fake.setpoints);time.sleep(.3)
            assert len(fake.setpoints)==points
            assert state()['mission']['state']=='stopped'
            print('PASS: airborne forced disarm, active mission cancelled, no subsequent LAND/HOLD setpoints; PTY only')
        finally:
            if client:client.stop()
            if process:
                process.terminate();process.communicate(timeout=8)
            fake.close()


if __name__=='__main__':run()
