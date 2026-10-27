"""HTTP adapter only. C++ owns mission execution and continues without Python."""
from threading import RLock
from .cpp_bridge import ControlError


class MissionClient:
    def __init__(self, console):
        self.client = console.client
        self.command_gate = RLock()

    @property
    def active(self):
        return self.client.status()['data'].get('mission', {}).get('active', False)

    def _request(self, command, params=None, client='mission-api-reader'):
        result = self.client.request(command, params or {}, client)
        if not result.get('ok'):
            raise ControlError(result.get('message', 'C++任务接口失败'))
        return result

    def status(self):
        return self._request('MISSION_STATUS')['mission']

    def start(self, document, client):
        return self._request('MISSION_START', document, client)

    def stop(self, mode='hold', client=None):
        return self._request('MISSION_STOP', {'mode': mode}, client or 'mission-api-stop')

    def emergency_stop(self, client):
        return self._request('EMERGENCY_STOP', client=client)
