import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mavlink_lite import MESSAGES, MavlinkEncoder, MavlinkParser, crc_x25  # noqa: E402

# Field definitions from MAVLink common.xml, in WIRE order (largest type first).
# CRC_EXTRA is derived from these exactly as pymavlink's generator does it.
DEFS = {
    "HEARTBEAT": [("uint32_t", "custom_mode"), ("uint8_t", "type"), ("uint8_t", "autopilot"),
                  ("uint8_t", "base_mode"), ("uint8_t", "system_status"), ("uint8_t", "mavlink_version")],
    "SYS_STATUS": [("uint32_t", "onboard_control_sensors_present"), ("uint32_t", "onboard_control_sensors_enabled"),
                   ("uint32_t", "onboard_control_sensors_health"), ("uint16_t", "load"),
                   ("uint16_t", "voltage_battery"), ("int16_t", "current_battery"),
                   ("uint16_t", "drop_rate_comm"), ("uint16_t", "errors_comm"), ("uint16_t", "errors_count1"),
                   ("uint16_t", "errors_count2"), ("uint16_t", "errors_count3"), ("uint16_t", "errors_count4"),
                   ("int8_t", "battery_remaining")],
    "GPS_RAW_INT": [("uint64_t", "time_usec"), ("int32_t", "lat"), ("int32_t", "lon"), ("int32_t", "alt"),
                    ("uint16_t", "eph"), ("uint16_t", "epv"), ("uint16_t", "vel"), ("uint16_t", "cog"),
                    ("uint8_t", "fix_type"), ("uint8_t", "satellites_visible")],
    "ATTITUDE": [("uint32_t", "time_boot_ms"), ("float", "roll"), ("float", "pitch"), ("float", "yaw"),
                 ("float", "rollspeed"), ("float", "pitchspeed"), ("float", "yawspeed")],
    "GLOBAL_POSITION_INT": [("uint32_t", "time_boot_ms"), ("int32_t", "lat"), ("int32_t", "lon"),
                            ("int32_t", "alt"), ("int32_t", "relative_alt"), ("int16_t", "vx"),
                            ("int16_t", "vy"), ("int16_t", "vz"), ("uint16_t", "hdg")],
    "VFR_HUD": [("float", "airspeed"), ("float", "groundspeed"), ("float", "alt"), ("float", "climb"),
                ("int16_t", "heading"), ("uint16_t", "throttle")],
}
FMT = {"uint64_t": "Q", "uint32_t": "I", "int32_t": "i", "float": "f", "uint16_t": "H", "int16_t": "h",
       "uint8_t": "B", "int8_t": "b"}


def test_crc_check_value():
    assert crc_x25(b"123456789") == 0x6F91          # CRC-16/MCRF4XX standard check value


@pytest.mark.parametrize("msg_id", sorted(MESSAGES))
def test_crc_extra_and_layout_match_common_xml(msg_id):
    name, fmt, names, extra = MESSAGES[msg_id]
    fields = DEFS[name]
    c = crc_x25((name + " ").encode())
    for t, n in fields:
        c = crc_x25((t + " ").encode(), c)
        c = crc_x25((n + " ").encode(), c)
    assert extra == (c & 0xFF) ^ (c >> 8)
    assert names == tuple(n for _, n in fields)
    assert fmt == "<" + "".join(FMT[t] for t, _ in fields)


def test_roundtrip_all_messages_in_one_stream():
    enc, p = MavlinkEncoder(sysid=7, compid=1), MavlinkParser()
    stream = b"".join([
        enc.encode("HEARTBEAT", type=2, autopilot=12, base_mode=129, system_status=4, mavlink_version=3),
        enc.encode("GLOBAL_POSITION_INT", time_boot_ms=1234, lat=336564523, lon=730164098, alt=590000,
                   relative_alt=80000, vx=-120, vy=250, vz=-30, hdg=4500),
        enc.encode("VFR_HUD", airspeed=12.5, groundspeed=12.0, alt=590.0, climb=0.3, heading=45, throttle=55),
        enc.encode("GPS_RAW_INT", time_usec=10**12, lat=1, lon=2, alt=3, fix_type=3, satellites_visible=14),
    ])
    msgs = []
    for i in range(0, len(stream), 7):               # arrives in awkward chunks, like a serial port
        msgs += p.feed(stream[i:i + 7])
    assert [m.name for m in msgs] == ["HEARTBEAT", "GLOBAL_POSITION_INT", "VFR_HUD", "GPS_RAW_INT"]
    gp = msgs[1]
    assert (gp.lat, gp.lon, gp.relative_alt, gp.vx, gp.vy, gp.vz, gp.hdg) == \
        (336564523, 730164098, 80000, -120, 250, -30, 4500)
    assert gp.sysid == 7 and msgs[2].groundspeed == pytest.approx(12.0)
    assert p.bad_crc == 0


def test_zero_trimming_and_sequence():
    enc = MavlinkEncoder()
    a = enc.encode("ATTITUDE", time_boot_ms=5)       # all-zero floats get trimmed away
    assert a[1] < struct.calcsize(MESSAGES[30][1])
    b = enc.encode("ATTITUDE", time_boot_ms=5)
    assert a[4] == 0 and b[4] == 1                   # sequence number increments
    (m,) = MavlinkParser().feed(a)
    assert m.time_boot_ms == 5 and m.roll == 0.0


def test_corrupted_frame_is_rejected_and_parser_resyncs():
    enc, p = MavlinkEncoder(), MavlinkParser()
    good = enc.encode("VFR_HUD", groundspeed=3.0)
    bad = bytearray(enc.encode("VFR_HUD", groundspeed=4.0))
    bad[12] ^= 0xFF
    msgs = p.feed(b"\x00garbage" + bytes(bad) + good)
    assert [m.groundspeed for m in msgs] == [pytest.approx(3.0)]
    assert p.bad_crc >= 1


def test_mavlink_v1_frames_are_parsed():
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)           # HEARTBEAT from a GCS
    body = bytes([len(payload), 5, 255, 190, 0]) + payload
    crc = crc_x25(bytes([50]), crc_x25(body))
    (m,) = MavlinkParser().feed(bytes([0xFE]) + body + struct.pack("<H", crc))
    assert m.name == "HEARTBEAT" and m.type == 6 and m.sysid == 255


def test_cross_check_with_pymavlink_if_installed():
    common = pytest.importorskip("pymavlink.dialects.v20.common")
    mav = common.MAVLink(None, srcSystem=1, srcComponent=1)
    ours = MavlinkEncoder().encode("GLOBAL_POSITION_INT", time_boot_ms=1, lat=2, lon=3, alt=4,
                                   relative_alt=5, vx=6, vy=7, vz=8, hdg=9)
    m = mav.parse_char(ours)
    assert m is not None and m.get_type() == "GLOBAL_POSITION_INT" and m.relative_alt == 5
    theirs = common.MAVLink_vfr_hud_message(1.0, 2.0, 3, 4, 5.0, 6.0).pack(mav)
    (back,) = MavlinkParser().feed(theirs)
    assert back.groundspeed == pytest.approx(2.0) and back.heading == 3
