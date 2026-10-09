"""Micolink protocol client for MicoAir MTF-01 / MTF-02 range + flow modules.

Implements the frame format, device identification, the tof_flow parameter
page and the streaming RANGE_SENSOR (msg 81) payload as described in the
repository README. The firmware update protocol is not implemented.

Verified against a real MTF-01 (hardware 10, firmware 1.1.0): frame format,
heartbeat identify, RANGE_SENSOR payload, GET_PARAMS_1 command and CMD_ACK.
SET_PARAMS_1 and RESET_DEFAULTS follow the same framing; their device
response was not verified on that unit.

No third-party dependencies are used except pyserial in __main__.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass

FRAME_HEAD = 0xEF
MAX_PAYLOAD = 64

DEV_PC_GCS = 1
DEV_RANGE_SENSOR = 15
DEV_OPTICAL_FLOW = 20

MSG_HEARTBEAT = 1
MSG_CMD = 2
MSG_CMD_ACK = 3
MSG_RANGE_SENSOR = 81   # 0x51
MSG_OPTICAL_FLOW = 85   # 0x55
MSG_VIO = 86

CMD_GET_PARAMS_1 = 1
CMD_SET_PARAMS_1 = 2
CMD_GET_PARAMS_2 = 3
CMD_SET_PARAMS_2 = 4
CMD_GET_PARAMS_3 = 5
CMD_SET_PARAMS_3 = 6
CMD_GET_PARAMS_4 = 7
CMD_SET_PARAMS_4 = 8
CMD_RESET_DEFAULTS = 48

PRODUCT_MODELS = {1: 'MT-01P', 2: 'MTF-01', 3: 'MTF-02', 6: 'MT-06', 16: 'MT-15'}
FLOW_CAPABLE = frozenset((2, 3))

OUTPUT_PROTOCOL = {0: 'MicoLink', 1: 'MSP', 2: 'MAVLink (ArduPilot)', 3: 'MAVLink (PX4)', 4: 'AUTO'}
FLOW_DIRECTION = {0: 'none', 1: '90 deg', 2: '180 deg', 3: '270 deg'}
RANGE_FREQ = {0: 800, 1: 400, 2: 200, 3: 100, 4: 50, 5: 25, 6: 10, 7: 5}   # Hz by index
RNGFND_ORIENT = {0: 'FORWARD', 1: 'FORWARD_RIGHT', 2: 'RIGHT', 3: 'BACK_RIGHT', 4: 'BACK',
                 5: 'BACK_LEFT', 6: 'LEFT', 7: 'FORWARD_LEFT', 8: 'UP', 9: 'DOWN'}
INTERFACE_TYPE = {0: 'UART', 1: 'I2C', 2: 'I2C_NOOPLOOP'}


def build_frame(dev_id: int, sys_id: int, msg_id: int, payload: bytes, seq: int) -> bytes:
    """Assemble one Micolink frame (header, payload, checksum)."""
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'payload longer than {MAX_PAYLOAD} bytes')
    body = bytes([FRAME_HEAD, dev_id, sys_id & 0xFF, msg_id, seq & 0xFF, len(payload)]) + payload
    return body + bytes([sum(body) & 0xFF])


class MicolinkParser:
    """Resyncing frame parser: scan for 0xEF, verify checksum, collect frames.

    Yields tuples of (dev_id, sys_id, msg_id, seq, payload).
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)
        frames = []
        while self._buf:
            if self._buf[0] != FRAME_HEAD:
                self._buf.pop(0)
                continue
            if len(self._buf) < 6:
                break
            length = self._buf[5]
            if length > MAX_PAYLOAD:
                self._buf.pop(0)
                continue
            total = 7 + length
            if len(self._buf) < total:
                break
            frame = bytes(self._buf[:total])
            if sum(frame[:-1]) & 0xFF == frame[-1]:
                frames.append((frame[1], frame[2], frame[3], frame[4],
                               bytes(frame[6:-1])))
                del self._buf[:total]
            else:
                self._buf.pop(0)
        return frames


def host_heartbeat_payload(model_hint: int = 30, time_ms: int | None = None) -> bytes:
    """13-byte host heartbeat: '<IBBBBBI' (time, model hint, 0, 1, 0, 1, 0)."""
    if time_ms is None:
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
    return struct.pack('<IBBBBBI', time_ms & 0xFFFFFFFF, model_hint & 0xFF, 0, 1, 0, 1, 0)


@dataclass
class DeviceID:
    """HEARTBEAT reply: identity of a module."""
    sys_id: int
    system_time_ms: int
    product_model: int
    hw_version: int
    fw_version: tuple[int, int, int]

    @property
    def product_name(self) -> str:
        return PRODUCT_MODELS.get(self.product_model, f'model {self.product_model}')

    @classmethod
    def from_frame(cls, dev_id: int, sys_id: int, payload: bytes) -> 'DeviceID | None':
        if dev_id != DEV_RANGE_SENSOR or len(payload) < 13:
            return None
        system_time, model, hw = struct.unpack_from('<IBB', payload, 0)
        fw = (payload[8], payload[7], payload[6])
        return cls(sys_id, system_time, model, hw, fw)


@dataclass
class RangeFlow:
    """RANGE_SENSOR (msg 81) payload, 20 bytes; 0 distance = no valid reading."""
    time_ms: int
    distance_mm: int          # 0 = no valid reading, 10 mm = smallest effective
    strength: int
    precision: int
    status: int               # 1 = distance usable
    flow_vel_x: int           # cm/s at 1 m; ground speed = value * height_m
    flow_vel_y: int
    flow_quality: int
    flow_status: int          # 1 = flow usable

    @classmethod
    def from_payload(cls, payload: bytes) -> 'RangeFlow | None':
        if len(payload) < 20:
            return None
        fields = struct.unpack('<IIBBBBhhBB', payload[:18])
        # offsets 11 and 18-19 are reserved and skipped
        return cls(time_ms=fields[0], distance_mm=fields[1], strength=fields[2],
                   precision=fields[3], status=fields[4],
                   flow_vel_x=fields[6], flow_vel_y=fields[7],
                   flow_quality=fields[8], flow_status=fields[9])


@dataclass
class TofFlowParams:
    """tof_flow parameter page (GET/SET_PARAMS_1), 12 bytes.

    Field model per the configuration tool's parse; outputProtocol,
    interfaceType and the MAVLink ids verified as factory values on an
    MTF-01, other fields unverified on device.
    """
    output_protocol: int = 4          # OUTPUT_PROTOCOL; 4 = AUTO (factory)
    uart_baud_rate: int = 0           # encoding undocumented; tested unit reads 0 on 115200
    flow_direction: int = 0           # FLOW_DIRECTION, image rotation
    flow_scale: int = 100             # 0.01 steps: 100 = 1.00, tool range 1..10000
    id: int = 0                       # 0..31
    range_freq: int = 0               # RANGE_FREQ index, 800 Hz at 0
    rngfnd_orient: int = 0            # RNGFND_ORIENT
    interface_type: int = 0           # INTERFACE_TYPE; 0 = UART
    query_mode: int = 0               # meaning undocumented (20 observed on a factory unit)
    mav_sys_id: int = 1
    mav_comp_id: int = 88

    @classmethod
    def from_ack(cls, params: bytes) -> 'TofFlowParams | None':
        if len(params) < 12:
            return None
        vals = struct.unpack_from('<BBBHBBBBBBB', params, 0)
        return cls(*vals)

    def to_bytes(self) -> bytes:
        return struct.pack('<BBBHBBBBBBB',
                           self.output_protocol, self.uart_baud_rate, self.flow_direction,
                           self.flow_scale, self.id, self.range_freq, self.rngfnd_orient,
                           self.interface_type, self.query_mode, self.mav_sys_id, self.mav_comp_id)

    def describe(self) -> str:
        return (f'outputProtocol={OUTPUT_PROTOCOL.get(self.output_protocol, self.output_protocol)} '
                f'flowDirection={FLOW_DIRECTION.get(self.flow_direction, self.flow_direction)} '
                f'flowScale={self.flow_scale * 0.01:g} '
                f'rangeFreq={RANGE_FREQ.get(self.range_freq, self.range_freq)}Hz '
                f'orient={RNGFND_ORIENT.get(self.rngfnd_orient, self.rngfnd_orient)} '
                f'interface={INTERFACE_TYPE.get(self.interface_type, self.interface_type)} '
                f'mav={self.mav_sys_id}/{self.mav_comp_id} '
                f'baud={self.uart_baud_rate} queryMode={self.query_mode} id={self.id}')


def cmd_payload(cmd_id: int, params: bytes = b'', pad_to: int = 18) -> bytes:
    """CMD frame payload: u16 LE command id + params, zero-padded (tool form)."""
    data = struct.pack('<H', cmd_id) + bytes(params)
    if len(data) < pad_to:
        data += bytes(pad_to - len(data))
    return data


def cmd_get_params_1() -> bytes:
    """GET_PARAMS_1 CMD payload (device response verified)."""
    return cmd_payload(CMD_GET_PARAMS_1, b'')


def cmd_set_params_1(params: TofFlowParams) -> bytes:
    """SET_PARAMS_1 CMD payload; device response unverified on the tested unit."""
    return cmd_payload(CMD_SET_PARAMS_1, params.to_bytes())


def cmd_reset_defaults() -> bytes:
    """RESET_DEFAULTS broadcast CMD payload; device response unverified."""
    return cmd_payload(CMD_RESET_DEFAULTS, b'')


class Client:
    """Frame-level Micolink client over an open serial port (pyserial or similar).

    The port object needs .write(), .read(n) and a small timeout, like
    serial.Serial.
    """

    def __init__(self, port, host_dev_id: int = DEV_PC_GCS) -> None:
        self._port = port
        self._host = host_dev_id
        self._seq = 0
        self._parser = MicolinkParser()

    def send(self, sys_id: int, msg_id: int, payload: bytes) -> None:
        self._port.write(build_frame(self._host, sys_id, msg_id, payload, self._seq))
        self._seq = (self._seq + 1) & 0xFF

    def poll(self, seconds: float = 0.05) -> list[tuple]:
        """Read available bytes, return parsed frames."""
        return self._parser.feed(self._port.read(8192))

    def identify(self, timeout: float = 10.0, flood: bool = True) -> DeviceID:
        """Identify by host heartbeats.

        The vendor tool sends heartbeats at 1 Hz; the tested MTF-01 answered
        only under a rapid burst, so flood cadence is the default here.
        """
        deadline = time.monotonic() + timeout
        next_hb = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hb:
                self.send(0, MSG_HEARTBEAT, host_heartbeat_payload())
                next_hb = now + (0.02 if flood else 1.0)
            for frame in self.poll(0.02):
                dev_id, dev_sys_id, msg_id, _, payload = frame
                ident = DeviceID.from_frame(dev_id, dev_sys_id, payload)
                if ident is not None:
                    return ident
        raise TimeoutError('no heartbeat reply from the module')

    def get_params_1(self, sys_id: int, timeout: float = 2.0,
                     heartbeat_hz: float = 50.0) -> TofFlowParams:
        """Read the tof_flow parameter page; needs heartbeat traffic alongside."""
        deadline = time.monotonic() + timeout
        next_hb = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_hb:
                self.send(0, MSG_HEARTBEAT, host_heartbeat_payload())
                next_hb = now + (1.0 / heartbeat_hz if heartbeat_hz else 999)
            self.send(sys_id, MSG_CMD, cmd_get_params_1())
            for frame in self.poll(0.02):
                dev_id, dev_sys_id, msg_id, _, payload = frame
                if msg_id == MSG_CMD_ACK and len(payload) >= 2:
                    cmd = struct.unpack_from('<H', payload)[0]
                    if cmd == CMD_GET_PARAMS_1:
                        params = TofFlowParams.from_ack(payload[2:])
                        if params is not None:
                            return params
        raise TimeoutError('no CMD_ACK for GET_PARAMS_1')

    def set_params_1(self, sys_id: int, params: TofFlowParams, timeout: float = 2.0,
                     heartbeat_hz: float = 50.0) -> TofFlowParams:
        """Write the tof_flow parameter page and re-read it to confirm.

        There is no separate save command; the vendor tool writes, waits about
        300 ms, then re-reads. Device response unverified on the tested unit.
        """
        self.send(sys_id, MSG_CMD, cmd_set_params_1(params))
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            self.poll(0.02)
        time.sleep(0.3)
        return self.get_params_1(sys_id, timeout=timeout, heartbeat_hz=heartbeat_hz)

    def stream(self, duration: float, sys_id: int | None = None,
               callback=None) -> list[RangeFlow]:
        """Print (or callback) RANGE_SENSOR frames for a duration."""
        readings: list[RangeFlow] = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for frame in self.poll(0.02):
                dev_id, dev_sys_id, msg_id, _, payload = frame
                if msg_id != MSG_RANGE_SENSOR or (sys_id is not None and dev_sys_id != sys_id):
                    continue
                reading = RangeFlow.from_payload(payload)
                if reading is None:
                    continue
                readings.append(reading)
                if callback is not None:
                    callback(reading)
        return readings


def _main() -> None:
    import sys

    if len(sys.argv) < 2:
        print(f'usage: {sys.argv[0]} /dev/ttyUSB0 [baud=115200]')
        raise SystemExit(2)
    device = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    import serial
    with serial.Serial(device, baudrate=baud, timeout=0.02) as port:
        client = Client(port)
        ident = client.identify(timeout=10.0)
        print(f'identified: {ident.product_name} sys_id={ident.sys_id} '
              f'hw={ident.hw_version} fw={ident.fw_version[0]}.{ident.fw_version[1]}.{ident.fw_version[2]}')
        try:
            params = client.get_params_1(ident.sys_id)
            print('params:', params.describe())
        except TimeoutError as exc:
            print(f'params unavailable: {exc}')

        def show(r: RangeFlow) -> None:
            print(f't={r.time_ms:8d}ms dist={r.distance_mm:5d}mm status={r.status} '
                  f'flow=({r.flow_vel_x}, {r.flow_vel_y}) q={r.flow_quality} fs={r.flow_status}')
        client.stream(10.0, callback=show)


if __name__ == '__main__':
    _main()