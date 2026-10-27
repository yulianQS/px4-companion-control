"""Defaults persistence/API restart tests; no camera, network board, or flight service."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from web.server.vision_control_config import DEFAULTS, VisionControlConfig
from web.server.http import Handler, ThreadingHTTPServer

class DefaultsTests(unittest.TestCase):
    def test_legacy_three_axis_file_keeps_values(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'vision_control.json'
            legacy={key:value for key,value in DEFAULTS.items() if not key.startswith('lateral_')}
            legacy['yaw_gain']=1.23
            path.write_text(json.dumps(dict(version=1,parameters=legacy)))
            values=VisionControlConfig(path).load()['parameters']
            self.assertEqual(values['yaw_gain'],1.23)
            self.assertFalse(values['lateral_enabled'])

    def test_saved_parameters_survive_new_instance_and_failed_write(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'config/vision_control.json'
            store=VisionControlConfig(path)
            self.assertFalse(store.load()['stored'])
            values=dict(DEFAULTS,yaw_gain=.8,yaw_ki=.02,forward_speed_m_s=.4,target_distance_m=1.5)
            store.save(values)
            self.assertEqual(VisionControlConfig(path).load()['parameters'],values)
            original=path.read_bytes()
            with patch('web.server.vision_control_config.os.replace',side_effect=OSError('test disk error')):
                with self.assertRaises(OSError):store.save(dict(values,yaw_gain=.9))
            self.assertEqual(path.read_bytes(),original)
            self.assertFalse(list(path.parent.glob('*.tmp')))
            for change in [dict(yaw_gain=float('nan')),dict(yaw_gain=True),dict(tag_size_m=0),dict(forward_speed_m_s=-1)]:
                with self.assertRaises(ValueError):store.save(dict(values,**change))
            self.assertEqual(store.load()['parameters'],values)

    def test_http_save_and_server_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'vision_control.json'
            def serve():
                server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
                server.daemon_threads=True
                server.vision_control_config=VisionControlConfig(path)
                # No console attached: saving defaults must never need a flight connection.
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
                return server,thread,'http://127.0.0.1:'+str(server.server_port)+'/api/vision/control-config'
            server,thread,url=serve()
            values=dict(DEFAULTS,yaw_gain=.8,distance_gain=.5,vertical_ki=.01)
            try:
                self.assertFalse(json.load(urlopen(url))['stored'])
                request=Request(url,json.dumps(values).encode(),{'Content-Type':'application/json','X-Control-Request':'1'},method='POST')
                self.assertEqual(json.load(urlopen(request))['parameters'],values)
                bad=Request(url,b'{}',{'Content-Type':'application/json','X-Control-Request':'1'},method='POST')
                with self.assertRaises(HTTPError) as error:urlopen(bad)
                self.assertEqual(error.exception.code,400)
            finally:server.shutdown();server.server_close();thread.join(2)
            server,thread,url=serve()
            try:
                loaded=json.load(urlopen(url));self.assertTrue(loaded['stored'])
                self.assertEqual(loaded['parameters'],values)
                path.write_text('{broken')
                with self.assertRaises(HTTPError) as error:urlopen(url)
                self.assertEqual(error.exception.code,503)
            finally:server.shutdown();server.server_close();thread.join(2)

if __name__=='__main__':unittest.main()
