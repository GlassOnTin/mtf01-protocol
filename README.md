# MTF-01 protocol reference

This is a protocol reference for the MicoAir MTF-01 (and the closely related
MTF-02) laser rangefinder + optical flow modules, for firmware and tool
authors who want to talk to the module directly instead of using the vendor's
configuration tool.

This document is unofficial. MicoAir has not reviewed or endorsed it. The
streaming frame format also appears in MicoAir's own published sample code and
Micolink pages; the configuration command set documented here comes from
protocol observation and is not covered by that material. Where a statement
has been checked against real hardware it is marked **verified**; everything
else is marked as unverified. See
[Field verification summary](#field-verification-summary).

Micolink and MSP are also the names of unrelated projects (MicoAir's protocol,
the MultiWii Serial Protocol). The names are used here as MicoAir uses them.

## Hardware summary

| | MTF-01 / MTF-02 |
|---|---|
| Link | UART, 3.3 V LVTTL, full duplex, 8N1 |
| Default baud rate | 115200 (verified: tested unit streams at 115200) |
| Ranging | ToF, 0.02 to 8 m, 80 mm minimum range, 850 nm emitter |
| Flow | optical-flow camera, 42° FOV, needs > 60 lux |
| Output rate | up to 100 Hz |

The flow velocity values are cm/s at 1 m of altitude. Ground speed in cm/s is
`flow * height_m`.

Ranges and reflectivity: the 850 nm emitter needs surfaces that reflect well.
Dark asphalt is roughly 10% reflective, so plan usable range well below 8 m
over such surfaces.

## Frame format (Micolink)

Every frame on the wire, both directions:

| byte | field | value |
|---|---|---|
| 0 | frame head | 0xEF |
| 1 | dev_id | device class of the sender |
| 2 | sys_id | system id of the sender (addressing, see below) |
| 3 | msg_id | message type |
| 4 | seq | increments per frame, mod 256 |
| 5 | len | payload length |
| 6 .. 6+len-1 | payload | |
| 6+len | checksum | u8 sum of every preceding byte, mod 256 |

Maximum payload length is 64 bytes. A parser can resync by scanning for 0xEF,
taking `len` from byte 5, and verifying the checksum; a frame with len > 64 is
discardable. **verified** against device frames.

## Message IDs

| msg_id | name | direction |
|---|---|---|
| 1 | HEARTBEAT | both |
| 2 | CMD | host to device |
| 3 | CMD_ACK | device to host |
| 81 (0x51) | RANGE_SENSOR | device to host, streaming data |
| 85 (0x55) | OPTICAL_FLOW | device to host (flow-only products) |
| 86 | VIO | device to host (MicoVIO products) |

For the MTF-01 and MTF-02 the ranging and flow data arrive together in
RANGE_SENSOR (81).

## Device addressing

`dev_id` is a device class, from MicoAir's published tables:

| dev_id | device | product names |
|---|---|---|
| 1 | PC GCS | (host / configuration tool) |
| 2 | mobile GCS | |
| 3 | FC | |
| 4 | gimbal | |
| 8 | ESC | |
| 15 | range sensor | MT-01P, MTF-01, MTF-02, MT-06, MT-15 |
| 20 | optical flow | FL01, MicoVIO |
| 21 | GPS | |
| 22 | UWB | |
| 23 | RTK | |
| 30 | telemetry radio | |
| 31 | RC receiver | |

On a 4-wire UART link the host sends frames with `dev_id` 1 (PC GCS) and a
`sys_id` naming the target: 0 for broadcast, otherwise the target's system
id. The device replies with `dev_id` 15 (range sensor) and its own `sys_id`.
An MTF-01 fresh from the factory replies with `sys_id` 0 (verified on the
tested unit), so a single module on a wire is addressable at sys_id 0 and the
module's outgoing identification frames also carry sys_id 0.

## Heartbeat and device identification

The host sends HEARTBEAT (msg 1) as a 13-byte payload:

| offset | type | field |
|---|---|---|
| 0 | u32 LE | time in ms |
| 4 | u8 | device model hint (the vendor tool sends 30 before identification) |
| 5 | u8 | 0 |
| 6 | u8 | 1 |
| 7 | u8 | 0 |
| 8 | u8 | 1 |
| 9 | u32 LE | 0 |

The vendor tool sends heartbeats at 1 Hz from connect until shutdown. The
tested MTF-01 answered only when heartbeat frames arrived in a rapid burst
(about 50 per second for a few seconds) and stayed silent at 1 Hz. See the
quirks section.

A device HEARTBEAT reply identifies the module. 13-byte payload:

| offset | type | field |
|---|---|---|
| 0 | u32 LE | systemTime, ms since module start |
| 4 | u8 | productModel: 1 MT-01P, 2 MTF-01, 3 MTF-02, 6 MT-06, 16 MT-15 |
| 5 | u8 | hardware version |
| 6 | 3 B | firmware version |
| 9 | | MicoVIO units extend the payload; not covered here |

Read the firmware version as `byte8.byte7.byte6`, so a unit on firmware 1.1.0
sends `00 01 01`. **verified**: the tested unit (productModel 2, hw 10,
firmware 1.1.0) replied with a correctly decoded payload, and productModel 2
means MTF-01.

## Streaming data (msg 81)

RANGE_SENSOR frames stream continuously once the module outputs Micolink
(one-directional, no host request needed). 20-byte payload, from MicoAir's
published struct definition, **verified** field-for-field on device frames:

| offset | type | field |
|---|---|---|
| 0 | u32 LE | time_ms, module system time |
| 4 | u32 LE | distance in mm; 0 = no valid reading, 10 mm = smallest effective value |
| 8 | u8 | strength, signal strength |
| 9 | u8 | precision, distance precision |
| 10 | u8 | dis_status, 1 = distance usable |
| 11 | u8 | reserved |
| 12 | i16 LE | flow_vel_x, cm/s at 1 m |
| 14 | i16 LE | flow_vel_y, cm/s at 1 m |
| 16 | u8 | flow_quality |
| 17 | u8 | flow_status, 1 = flow usable |
| 18 | u16 LE | reserved |

The 80 mm minimum range means targets closer than that return no reading.
Distance 0 is the "no reading" marker, and dis_status 1 is the usable signal.

## CMD / CMD_ACK (msg 2, 3) and the configuration command set

A CMD payload is a u16 LE command id followed by parameter bytes. The vendor
tool zero-pads the whole CMD payload to 18 bytes. A CMD_ACK payload is a u16
LE echo of the command id followed by the response parameters.

| cmd_id | name | notes |
|---|---|---|
| 1 | GET_PARAMS_1 | read the tof_flow parameter page |
| 2 | SET_PARAMS_1 | write the tof_flow parameter page |
| 3 | GET_PARAMS_2 | read page 2 (telemetry radio products) |
| 4 | SET_PARAMS_2 | write page 2 |
| 5 | GET_PARAMS_3 | read page 3 (telemetry radio products) |
| 6 | SET_PARAMS_3 | write page 3 |
| 7 | GET_PARAMS_4 | read page 4 |
| 8 | SET_PARAMS_4 | write page 4 |
| 48 | RESET_DEFAULTS | restore factory configuration, sent as a broadcast (sys_id 0), unverified |

Framing of the CMD and of the ACK echo is **verified** (GET_PARAMS_1 against
the tested unit). There is no separate save or persist command in this set;
the vendor tool's save flow is SET_PARAMS_1 followed ~300 ms later by a
GET_PARAMS_1 re-read to confirm the values stuck.

Pages 2 and 3 belong to the telemetry radio products: page 1 for a radio is
workMode, airSpeed, txPower, freqChannel, moduleAddr (u16), uartBaudRate,
retransmit; page 2 continues with workMode, airSpeed, txPower, freqChannel,
moduleAddr (u16), freqMode, usbBaudRate. Those layouts are not in scope for
the rangefinders.

## Parameters, tof_flow page (GET/SET_PARAMS_1)

12-byte payload for the MTF-01/MTF-02, field order from the vendor tool's
parser. The `outputProtocol`, `interfaceType` and `mavSysId`/`mavCompId`
values below were **verified** as the tested unit's factory configuration;
other rows describe the tool's model of the field, **unverified** on device.

| offset | type | field | tested unit (factory) |
|---|---|---|---|
| 0 | u8 | outputProtocol | 4 (AUTO) |
| 1 | u8 | uartBaudRate | 0 |
| 2 | u8 | flowDirection | 0 |
| 3 | u16 LE | flowScale | 100 |
| 5 | u8 | id | 0 |
| 6 | u8 | rangeFreq | 0 |
| 7 | u8 | rngfndOrient | 0 |
| 8 | u8 | interfaceType | 0 (UART) |
| 9 | u8 | queryMode | 20 |
| 10 | u8 | mavSysId | 1 |
| 11 | u8 | mavCompId | 88 |

Responses pad the payload to 16 bytes; the trailing padding bytes changed
between reads on the tested unit, so parse by field order and ignore padding.

### Field values

`outputProtocol`, 5 choices:

| value | mode |
|---|---|
| 0 | MicoLink (native streaming, msg 81) |
| 1 | MSP |
| 2 | MAVLink (ArduPilot) |
| 3 | MAVLink (PX4) |
| 4 | AUTO (factory) |

With outputProtocol AUTO the module chooses its output format from the
traffic it sees: an ArduPilot MSP client connected to the UART receives MSP
frames, and a burst of Micolink heartbeats switched the tested unit into
Micolink streaming. Both transitions were observed on device.

`flowDirection` is an image rotation for the flow camera: 0 = none, 1 = 90°,
2 = 180°, 3 = 270°.

`flowScale` is a scale factor stored in 0.01 units; the factory value 100
means 1.00. The configuration tool accepts 1 to 10000.

`id` is a module id, 0 to 31 in the tool, exposed for the non-flow
rangefinder products.

`rangeFreq` selects the rangefinder sample rate by index: 0 = 800 Hz, 1 = 400
Hz, 2 = 200 Hz, 3 = 100 Hz, 4 = 50 Hz, 5 = 25 Hz, 6 = 10 Hz, 7 = 5 Hz. The
tool hides it on the MT-06.

`rngfndOrient` uses ArduPilot's rangefinder orientation list as shown by the
tool: 0 FORWARD, 1 FORWARD_RIGHT, 2 RIGHT, 3 BACK_RIGHT, 4 BACK, 5 BACK_LEFT,
6 LEFT, 7 FORWARD_LEFT, 8 UP, 9 DOWN. The tool exposes it on the non-flow
products; an MTF-01 is normally mounted facing straight down.

`interfaceType`: 0 = UART, 1 = I2C, 2 = I2C_NOOPLOOP.

`queryMode` has no control in the configuration tool; 20 was observed. Its
meaning is unknown.

`uartBaudRate` maps to the tool's connection-rate list (38400, 57600, 115200,
230400, 460800, 921600) by an encoding documented nowhere; the tested unit
reads 0 while actually running 115200.

`mavSysId` and `mavCompId` are the MAVLink source system/component ids used
by the MAVLink output modes (values 1 and 88 observed at factory).

## MSP output mode

When the module outputs MSP (fixed mode 1, or AUTO with an MSP host), frames
are MSPv2:

| byte | field |
|---|---|
| 0..2 | 0x24, 0x58, 0x3C ('$', 'X', '<') |
| 3 | flags, u8 (0x00 observed) |
| 4 | function, u16 LE |
| 6 | payload length, u16 LE |
| 8.. | payload |
| last | CRC-8 DVB-S2 over bytes 3 through end of payload |

**verified** on device: CRC-valid frames at roughly 100 Hz combined
(rangefinder and flow frames interleaved).

Sensor payloads (these functions are the MSP sensor-client set used by
Betaflight and ArduPilot):

| function | name | payload | meaning |
|---|---|---|---|
| 0x1F01 | MSP2_SENSOR_RANGEFINDER | u8 quality, i32 LE distance_mm | negative distance = out of range; 5-byte payload **verified** |
| 0x1F02 | MSP2_SENSOR_OPTIC_FLOW | 9 bytes: flow x, y and a quality/status byte | i32/i32/u8 order per ArduPilot's parser; the tested unit sends all zeros, so byte order is not exercised by data **part verified** |

MAVLink output modes (2 and 3) exist in the parameter set; their frame
contents are not covered here.

## ArduPilot integration (MSP mode)

Configured on the flight controller, nothing needed on the module:

```
SERIALx_PROTOCOL = 32   # MSP
RNGFNDx_TYPE      = 32   # MSP rangefinder
FLOW_TYPE         = 7    # MSP optical flow
RNGFNDx_ORIENT    = 25   # DOWN for a bottom-facing MTF-01
RNGFNDx_MIN/MAX   = 0 / 8 # meters, spec 0.02-8 m with 80 mm minimum range
```

**verified**: an MTF-01 on this configuration fed the rangefinder and flow
backends on ArduPilot Copter 4.7.1.

## Firmware update

Firmware update uses a second frame format on the same UART (bootloader
handshake plus 0x55 0xAB framed bootloader commands and a downloadable
firmware image). The details are not documented here.

## Field verification summary

Verified on one MTF-01 (hardware 10, firmware 1.1.0):

- frame format, checksum, parser resync
- host heartbeat and device HEARTBEAT reply layout, product tables
- RANGE_SENSOR (81) 20-byte payload fields
- GET_PARAMS_1 command framing and CMD_ACK echo, tof_flow register values
- MSPv2 sensor frames (0x1F01 payload, 0x1F02 payload length)
- outputProtocol AUTO resolving to MSP, and switching to Micolink on host traffic

Unverified: SET_PARAMS_1 and RESET_DEFAULTS device behavior (SET was sent and
not acknowledged on the tested unit), uartBaudRate encoding, queryMode,
rangeFreq and id semantics, MAVLink output modes, MTF-02 / MT-01P / MT-06 /
MT-15 differences, MicoVIO payload, firmware update protocol.

Device quirks observed on the tested unit, in case they are unit-specific
hardware: the module answered identification and parameter reads only after a
rapid burst of heartbeat frames and stayed silent at the vendor tool's 1 Hz
cadence, and identification replies each carried a fresh systemTime, meaning
the module re-initialised at the protocol transition rather than answering
from a steady state.

## Related material

- `micolink.py`: a small dependency-free client implementing everything above
  except the firmware update protocol. `python3 micolink.py /dev/ttyUSB0`
  identifies a module and prints streaming frames.
- MicoAir's published MTF-01 sample code and user manual document the
  streaming frame from the vendor side.

## License

MIT, see LICENSE.