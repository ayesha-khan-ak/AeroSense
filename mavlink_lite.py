"""
AeroSense — mavlink_lite.py
A tiny, dependency-free MAVLink v2 (and v1 read) codec for the handful of
messages a ground station needs to show a drone on a map.

Why not pymavlink?  It works too (and the test-suite cross-checks against it
when installed), but this keeps the project installable on a laptop with
almost no free disk space, and it lets us explain the protocol in the viva:

  v2 frame = 0xFD | len | incompat | compat | seq | sysid | compid | msgid(3) |
             payload (trailing zeros trimmed) | CRC-16/MCRF4XX (2, little-endian)
  CRC covers everything after 0xFD + the message's CRC_EXTRA byte.

The CRC_EXTRA values below are computed from the message definitions in
common.xml and are verified in tests/test_mavlink_lite.py.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

STX_V2 = 0xFD
STX_V1 = 0xFE

# msg_id: (name, struct format of the wire payload, field names, crc_extra)
MESSAGES = {
    0:  ("HEARTBEAT", "<IBBBBB",
         ("custom_mode", "type", "autopilot", "base_mode", "system_status", "mavlink_version"), 50),
    1:  ("SYS_STATUS", "<IIIHHhHHHHHHb",
         ("onboard_control_sensors_present", "onboard_control_sensors_enabled",
          "onboard_control_sensors_health", "load", "voltage_battery", "current_battery",
          "drop_rate_comm", "errors_comm", "errors_count1", "errors_count2", "errors_count3",
          "errors_count4", "battery_remaining"), 124),
    24: ("GPS_RAW_INT", "<QiiiHHHHBB",
         ("time_usec", "lat", "lon", "alt", "eph", "epv", "vel", "cog", "fix_type",
          "satellites_visible"), 24),
    30: ("ATTITUDE", "<Iffffff",
         ("time_boot_ms", "roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"), 39),
    33: ("GLOBAL_POSITION_INT", "<IiiiihhhH",
         ("time_boot_ms", "lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg"), 104),
    74: ("VFR_HUD", "<ffffhH",
         ("airspeed", "groundspeed", "alt", "climb", "heading", "throttle"), 20),
}
NAME_TO_ID = {v[0]: k for k, v in MESSAGES.items()}

# A few enum values we use
MAV_TYPE_QUADROTOR = 2
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_PX4 = 12
MAV_AUTOPILOT_INVALID = 8
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_STATE_ACTIVE = 4


def crc_x25(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/MCRF4XX as used by MAVLink (check value for b'123456789' is 0x6F91)."""
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


@dataclass
class Message:
    msg_id: int
    name: str
    sysid: int
    compid: int
    fields: dict

    def __getattr__(self, item):            # msg.lat instead of msg.fields["lat"]
        try:
            return self.__dict__["fields"][item]
        except KeyError as e:
            raise AttributeError(item) from e


class MavlinkEncoder:
    def __init__(self, sysid: int = 1, compid: int = 1):
        self.sysid, self.compid, self.seq = sysid, compid, 0

    def encode(self, name: str, **values) -> bytes:
        msg_id = NAME_TO_ID[name]
        _, fmt, names, extra = MESSAGES[msg_id]
        payload = struct.pack(fmt, *[values.get(n, 0) for n in names])
        # MAVLink 2 trims trailing zero bytes of the payload (at least 1 byte kept)
        trimmed = payload.rstrip(b"\x00") or b"\x00"
        header = struct.pack("<BBBBBB", len(trimmed), 0, 0, self.seq, self.sysid, self.compid)
        header += struct.pack("<I", msg_id)[:3]
        crc = crc_x25(header + trimmed)
        crc = crc_x25(bytes([extra]), crc)
        self.seq = (self.seq + 1) & 0xFF
        return bytes([STX_V2]) + header + trimmed + struct.pack("<H", crc)


class MavlinkParser:
    """Feed raw bytes (UDP datagrams or a serial stream); get decoded Messages back."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0
        self.unknown = 0

    def feed(self, data: bytes) -> list[Message]:
        self.buf.extend(data)
        out = []
        while True:
            msg = self._parse_one()
            if msg is None:
                break
            if msg is not False:
                out.append(msg)
        return out

    def _parse_one(self):
        buf = self.buf
        # skip to the next start byte
        while buf and buf[0] not in (STX_V1, STX_V2):
            del buf[0]
        if len(buf) < 8:
            return None
        if buf[0] == STX_V2:
            plen, incompat = buf[1], buf[2]
            total = 10 + plen + 2 + (13 if incompat & 0x01 else 0)   # +signature
            if len(buf) < total:
                return None
            frame = bytes(buf[:total])
            seq, sysid, compid = frame[4], frame[5], frame[6]
            msg_id = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            payload = frame[10:10 + plen]
            crc_rx = struct.unpack("<H", frame[10 + plen:12 + plen])[0]
            crc_region = frame[1:10 + plen]
        else:
            plen = buf[1]
            total = 6 + plen + 2
            if len(buf) < total:
                return None
            frame = bytes(buf[:total])
            seq, sysid, compid, msg_id = frame[2], frame[3], frame[4], frame[5]
            payload = frame[6:6 + plen]
            crc_rx = struct.unpack("<H", frame[6 + plen:8 + plen])[0]
            crc_region = frame[1:6 + plen]

        if msg_id not in MESSAGES:
            # can't verify CRC without CRC_EXTRA: assume frame boundary is right and skip it
            self.unknown += 1
            del buf[:total]
            return False
        name, fmt, names, extra = MESSAGES[msg_id]
        if crc_x25(bytes([extra]), crc_x25(crc_region)) != crc_rx:
            self.bad_crc += 1
            del buf[0]                      # resync on the next start byte
            return False
        del buf[:total]
        size = struct.calcsize(fmt)
        payload = payload[:size].ljust(size, b"\x00")   # undo zero-trimming
        values = struct.unpack(fmt, payload)
        return Message(msg_id, name, sysid, compid, dict(zip(names, values)))
