"""Existing HTTP API compatibility; fake TCP facade, no flight hardware."""
import json
from pathlib import Path
import sys
import threading
import unittest
from urllib.request import Request,urlopen
from urllib.error import HTTPError
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from web.server.cpp_bridge import FlightConsole,ControlError
from web.server.flight_analysis import FlightArchive
from web.server.http import Handler,ThreadingHTTPServer

class FakeClient:
    def __init__(self):self.calls=[]
    def request(self,command,params,client,request_id=None):
        self.calls.append((command,params,client,request_id))
        return dict(ok=True,confirmed=False,stage='running',message='executing')
    def status(self):return dict(source='px4',data=dict(px4=dict(connected=False),control={},motion={},joystick={},trajectory={}))

class InterfaceTest(unittest.TestCase):
    def setUp(self):
        self.console=FlightConsole();self.console.client=FakeClient()
    def test_existing_action_maps_to_cpp(self):
        cases={'arm':'ARM','disarm':'DISARM','land':'LAND','takeoff':'TAKEOFF','task_start':'MOVE','task_cancel':'STOP_TRAJECTORY','mode':'SET_MODE','disable':'DISABLE','stick_start':'STICK_START'}
        for action,command in cases.items():
            result=self.console.perform('test-client',dict(action=action,request_id='request-123456',mode='OFFBOARD',task={'distance':.5}))
            self.assertEqual(self.console.client.calls[-1][0],command)
            self.assertFalse(result['confirmed'])
    def test_position_remains_cm_until_cpp(self):
        params=dict(frame='LOCAL_NED',unit='cm',x=50,y=0,z=-50)
        self.console.perform('test-client',dict(action='command',command='SET_POSITION',params=params,request_id='request-123456'))
        self.assertEqual(self.console.client.calls[-1][1]['x'],50)
    def test_no_arbitrary_mavlink_command(self):
        with self.assertRaises(ControlError):self.console.perform('test-client',dict(action='command',command='FORCE_ARM',request_id='request-123456'))
    def test_http_analysis_and_cross_origin(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        server.console=self.console;server.recorder=FlightArchive()
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            base='http://127.0.0.1:'+str(server.server_port)
            headers={'Content-Type':'application/json','X-Control-Request':'1'}
            body=json.dumps(dict(csv='actual_x,actual_y,actual_z,target_x,target_y,target_z\n1,0,0,0,0,0\n')).encode()
            with urlopen(Request(base+'/api/trajectory/analyze',data=body,headers=headers)) as response:
                self.assertEqual(json.load(response)['analysis']['rmse_3d_m'],1)
            with self.assertRaises(HTTPError):urlopen(Request(base+'/api/control/action',data=b'{}',headers=dict(headers,Origin='http://other')))
        finally:server.shutdown();server.server_close();thread.join()

if __name__=='__main__':unittest.main()
