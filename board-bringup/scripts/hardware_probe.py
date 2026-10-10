#!/usr/bin/env python3
"""Read-only mapping from live device-tree aliases to actual Linux tty nodes."""
from pathlib import Path
import json

root = Path('/sys/firmware/devicetree/base')
def text(path):
    return path.read_bytes().rstrip(b'\0').replace(b'\0', b' ').decode(errors='replace')

for alias in sorted((root / 'aliases').glob('serial*')):
    node = root / text(alias).lstrip('/')
    matches = [str(Path('/dev') / p.name) for p in Path('/sys/class/tty').glob('tty*')
               if (p / 'device/of_node').exists() and (p / 'device/of_node').resolve() == node.resolve()]
    print(json.dumps({'alias': alias.name, 'node': str(node),
                      'status': text(node / 'status') if (node / 'status').exists() else 'okay (implicit)',
                      'devices': matches}))
for p in sorted(root.glob('soc@3000000/*/*uart7*/*')):
    if p.name in ('pins', 'function'):
        print(str(p), text(p))
vin = root / 'soc@3000000/vind@5800800'
for p in sorted(vin.glob('sensor@*/*')):
    if p.name == 'status' or p.name.endswith('_mname'):
        print(str(p), text(p))
