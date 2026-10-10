#!/home/pi/venv/bin/python
"""Open verified UART7 and receive only. No write, flush, break or MAVLink commands."""
import argparse
from pathlib import Path
import time
import sys
import serial
from serial.tools import list_ports

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', help='Must map to live device-tree serial7')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--seconds', type=float, default=3.0)
    args = parser.parse_args()
    if args.baud <= 0 or not 0 <= args.seconds <= 60:
        parser.error('baud must be positive; seconds must be between 0 and 60')
    print('Enumerated ports:', [(p.device, p.description, p.hwid) for p in list_ports.comports()])
    dt = Path('/sys/firmware/devicetree/base')
    alias = dt / 'aliases/serial7'
    if not alias.exists():
        print('UART7_ALIAS_MISSING')
        return 2
    node = dt / alias.read_bytes().rstrip(b'\0').decode().lstrip('/')
    mapped = [Path('/dev') / p.name for p in Path('/sys/class/tty').glob('tty*')
              if (p / 'device/of_node').exists() and (p / 'device/of_node').resolve() == node.resolve()]
    print('UART7 node:', node, 'actual devices:', [str(p) for p in mapped])
    if len(mapped) != 1:
        print('UART7_NOT_AVAILABLE; no other serial ports were opened')
        return 2
    device = mapped[0]
    if args.device and Path(args.device).resolve() != device.resolve():
        print('REFUSED: requested device is not verified UART7')
        return 3
    if device.name in Path('/proc/cmdline').read_text():
        print('REFUSED: device is a kernel console')
        return 3
    port = serial.Serial(port=None, baudrate=args.baud, timeout=0.2, write_timeout=0.2,
                         bytesize=8, parity='N', stopbits=1, exclusive=True,
                         xonxoff=False, rtscts=False, dsrdtr=False)
    port.dtr = False
    port.rts = False
    port.port = str(device)
    try:
        port.open()
        print('OPEN_OK', device, args.baud, '8N1; receive-only', flush=True)
        deadline = time.monotonic() + args.seconds
        received = 0
        preview = bytearray()
        while time.monotonic() < deadline:
            data = port.read(256)
            received += len(data)
            if len(preview) < 64:
                preview.extend(data[:64 - len(preview)])
        print('RX_BYTES', received, 'PREVIEW_HEX', preview.hex())
        print('TX_BYTES 0; opening succeeds independently of whether a peer transmits.')
        return 0
    except (OSError, serial.SerialException) as exc:
        print('UART_ERROR:', exc)
        return 4
    finally:
        port.close()

if __name__ == '__main__':
    sys.exit(main())
