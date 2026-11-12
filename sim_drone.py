"""
AeroSense — sim_drone.py   (virtual drone / "SITL-lite")

Flies a QGroundControl .plan mission over FAST campus and broadcasts real
MAVLink v2 telemetry over UDP — exactly what a PX4/ArduPilot drone (or PX4
SITL) sends to a ground station.  Nothing has to be installed.

Where the packets go (pick one):

  A) Straight to the AeroSense server  (default, no QGC needed)
        python sim_drone.py
        -> udp 127.0.0.1:14445  -> server.py -> dashboard

  B) Through QGroundControl  (on any PC that has QGC, e.g. a lab PC)
        python sim_drone.py --target <QGC_PC_IP>:14550
        QGC auto-connects on UDP 14550 and shows the drone on its map.
        In QGC: Application Settings -> MAVLink -> "Enable MAVLink forwarding",
        host = <AeroSense_server_IP>:14445   -> QGC forwards to our server.

  C) Both at once (QGC display + our server)
        python sim_drone.py --target 127.0.0.1:14445 --target 127.0.0.1:14550

The server can also run this internally:  python server.py --sim
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from pathlib import Path

import geo
from mavlink_lite import (MavlinkEncoder, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
                          MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                          MAV_STATE_ACTIVE)
from qgc_plan import parse_plan

ROOT = Path(__file__).resolve().parent
DEFAULT_PLAN = ROOT / "data" / "missions" / "fast_intruder_demo.plan"
PX4_AUTO_MISSION = (4 << 16) | (4 << 24)      # PX4 custom_mode: main AUTO, sub MISSION


class SimDrone:
    def __init__(self, plan_path: Path = DEFAULT_PLAN, targets=("127.0.0.1:14445",),
                 rate_hz: float = 10.0, speed: float | None = None, loop: bool = True,
                 max_climb: float = 4.0, sysid: int = 1):
        self.plan_path = Path(plan_path)
        self.targets = [self._parse_target(t) for t in targets]
        self.rate_hz = rate_hz
        self.speed_override = speed
        self.loop = loop
        self.max_climb = max_climb
        self.enc = MavlinkEncoder(sysid=sysid, compid=1)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._plan_mtime = 0.0
        self.state = {}
        self._load_plan()

    @staticmethod
    def _parse_target(t: str):
        host, port = t.rsplit(":", 1)
        return host, int(port)

    def _load_plan(self):
        plan = parse_plan(json.loads(self.plan_path.read_text(encoding="utf-8")))
        if len(plan["waypoints"]) < 2:
            raise ValueError(f"{self.plan_path.name}: mission needs at least 2 waypoints")
        self.home = plan["home"]
        self.wps = plan["waypoints"]
        self.cruise = self.speed_override or plan["cruise_speed"]
        self._plan_mtime = self.plan_path.stat().st_mtime
        # start on the first waypoint, on the ground
        self.lat, self.lon = self.wps[0]["lat"], self.wps[0]["lon"]
        self.alt = 0.0
        self.target_idx = 0
        self.ve = self.vn = self.vu = 0.0
        self.heading = 0.0

    def stop(self):
        self._stop.set()

    # ------------------------------------------------------------ physics --
    def _step(self, dt: float):
        wp = self.wps[self.target_idx]
        e, n, u = geo.geodetic_to_enu(wp["lat"], wp["lon"], wp["alt"], self.lat, self.lon, self.alt)
        dist_h = math.hypot(e, n)
        if dist_h < 2.0 and abs(u) < 1.5:
            self.target_idx += 1
            if self.target_idx >= len(self.wps):
                if not self.loop:
                    self.target_idx = len(self.wps) - 1
                    self.ve = self.vn = self.vu = 0.0
                    return
                self.target_idx = 1           # skip the ground start point when looping
            return
        # time to finish the leg is limited by horizontal speed or climb rate
        leg_t = max(dist_h / self.cruise, abs(u) / self.max_climb, dt)
        tve, tvn, tvu = e / leg_t, n / leg_t, u / leg_t
        a = min(1.0, dt * 2.0)                # simple first-order velocity response
        self.ve += (tve - self.ve) * a
        self.vn += (tvn - self.vn) * a
        self.vu += (tvu - self.vu) * a
        self.lat, self.lon, self.alt = geo.enu_to_geodetic(self.ve * dt, self.vn * dt, self.vu * dt,
                                                           self.lat, self.lon, self.alt)
        self.alt = max(0.0, self.alt)
        if math.hypot(self.ve, self.vn) > 0.5:
            self.heading = (math.degrees(math.atan2(self.ve, self.vn)) + 360) % 360

    # ----------------------------------------------------------- messages --
    def _send(self, pkt: bytes):
        for addr in self.targets:
            try:
                self.sock.sendto(pkt, addr)
            except OSError:
                pass                          # nobody listening yet — that's fine for UDP

    def _emit(self, t_boot_ms: int, slow: bool):
        gs = math.hypot(self.ve, self.vn)
        lat_i, lon_i = int(self.lat * 1e7), int(self.lon * 1e7)
        alt_msl_mm = int((self.home[2] + self.alt) * 1000)
        if slow:
            self._send(self.enc.encode(
                "HEARTBEAT", custom_mode=PX4_AUTO_MISSION, type=MAV_TYPE_QUADROTOR,
                autopilot=MAV_AUTOPILOT_PX4,
                base_mode=MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                system_status=MAV_STATE_ACTIVE, mavlink_version=3))
            self._send(self.enc.encode(
                "SYS_STATUS", voltage_battery=15800, current_battery=1200, battery_remaining=
                max(5, 100 - int(t_boot_ms / 1000 / 12) % 95), load=350))
            self._send(self.enc.encode(
                "GPS_RAW_INT", time_usec=t_boot_ms * 1000, lat=lat_i, lon=lon_i, alt=alt_msl_mm,
                eph=80, epv=120, vel=int(gs * 100), cog=int(self.heading * 100), fix_type=3,
                satellites_visible=14))
        self._send(self.enc.encode(
            "GLOBAL_POSITION_INT", time_boot_ms=t_boot_ms, lat=lat_i, lon=lon_i, alt=alt_msl_mm,
            relative_alt=int(self.alt * 1000), vx=int(self.vn * 100), vy=int(self.ve * 100),
            vz=int(-self.vu * 100), hdg=int(self.heading * 100) % 36000))
        pitch = -math.atan2(gs, 40.0) * 0.6  # nose-down while flying forward (cosmetic)
        self._send(self.enc.encode(
            "ATTITUDE", time_boot_ms=t_boot_ms, roll=0.0, pitch=pitch,
            yaw=math.radians(self.heading if self.heading <= 180 else self.heading - 360)))
        self._send(self.enc.encode(
            "VFR_HUD", airspeed=gs, groundspeed=gs, alt=self.home[2] + self.alt, climb=self.vu,
            heading=int(self.heading), throttle=55))

    def run(self):
        dt = 1.0 / self.rate_hz
        t0 = time.monotonic()
        next_tick, last_slow, last_check = t0, 0.0, t0
        print(f"[sim_drone] flying {self.plan_path.name} ({len(self.wps)} waypoints) "
              f"-> {', '.join(f'{h}:{p}' for h, p in self.targets)}")
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_check > 2.0:        # hot-reload the mission when it is edited
                last_check = now
                try:
                    if self.plan_path.stat().st_mtime != self._plan_mtime:
                        self._load_plan()
                        print("[sim_drone] mission changed on disk — reloaded")
                except (OSError, ValueError) as e:
                    print("[sim_drone] could not reload mission:", e)
            self._step(dt)
            slow = now - last_slow >= 1.0
            if slow:
                last_slow = now
            self._emit(int((now - t0) * 1000), slow)
            self.state = {"lat": self.lat, "lon": self.lon, "alt": self.alt, "wp": self.target_idx}
            next_tick += dt
            self._stop.wait(max(0.0, next_tick - time.monotonic()))


def main():
    ap = argparse.ArgumentParser(description="AeroSense virtual drone (MAVLink over UDP)")
    ap.add_argument("--plan", default=str(DEFAULT_PLAN), help="QGroundControl .plan file to fly")
    ap.add_argument("--target", action="append",
                    help="host:port to send MAVLink to (repeatable). Default 127.0.0.1:14445")
    ap.add_argument("--speed", type=float, default=None, help="override cruise speed (m/s)")
    ap.add_argument("--once", action="store_true", help="fly the mission once instead of looping")
    args = ap.parse_args()
    drone = SimDrone(args.plan, args.target or ["127.0.0.1:14445"], speed=args.speed,
                     loop=not args.once)
    try:
        drone.run()
    except KeyboardInterrupt:
        print("\n[sim_drone] stopped")


if __name__ == "__main__":
    main()
