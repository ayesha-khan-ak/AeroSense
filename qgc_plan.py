"""
AeroSense — qgc_plan.py
Read and write QGroundControl ".plan" files (JSON) without installing QGC.

Format reference: QGroundControl Dev Guide → File Formats → Plan File Format.
  * mission.items[]       SimpleItem = one MAVLink MISSION_ITEM
                          params = [p1, p2, p3, p4, lat, lon, alt]
  * mission.plannedHomePosition = [lat, lon, alt_msl]
  * geoFence.polygons[]   {"inclusion": bool, "polygon": [[lat, lon], ...], "version": 1}
  * geoFence.circles[]    {"circle": {"center": [lat, lon], "radius": m}, "inclusion": bool, "version": 1}

So a mission drawn in our dashboard opens in QGroundControl, and a mission
drawn in QGroundControl (on any PC that has it) can be loaded by our
virtual drone / dashboard.
"""
from __future__ import annotations

from typing import Iterable

# MAV_CMD values used in simple missions
CMD_WAYPOINT = 16
CMD_RTL = 20
CMD_LAND = 21
CMD_TAKEOFF = 22
FRAME_GLOBAL_RELATIVE_ALT = 3
NAV_COMMANDS = {CMD_WAYPOINT, CMD_TAKEOFF, CMD_LAND}


def _simple_item(seq: int, command: int, lat: float, lon: float, alt: float) -> dict:
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": alt,
        "AltitudeMode": 1,                       # relative to home
        "autoContinue": True,
        "command": command,
        "doJumpId": seq,
        "frame": FRAME_GLOBAL_RELATIVE_ALT,
        "params": [0, 0, 0, None, lat, lon, alt],
        "type": "SimpleItem",
    }


def build_plan(waypoints: Iterable[dict], home: list, zones: Iterable = (),
               cruise_speed: float = 8.0, hover_speed: float = 5.0, takeoff: bool = True) -> dict:
    """
    waypoints: [{"lat":..,"lon":..,"alt":.. (m above home)}, ...]
    home:      [lat, lon, alt_msl]
    zones:     geofence.Zone objects; restricted/critical ones are exported
               as QGC *exclusion* polygons (keep-out areas).
    """
    wps = list(waypoints)
    items, seq = [], 1
    if takeoff and wps:
        items.append(_simple_item(seq, CMD_TAKEOFF, wps[0]["lat"], wps[0]["lon"], wps[0]["alt"]))
        seq += 1
    for w in wps:
        items.append(_simple_item(seq, CMD_WAYPOINT, w["lat"], w["lon"], w["alt"]))
        seq += 1

    polygons = []
    for z in zones:
        if getattr(z, "kind", "") in ("restricted", "critical"):
            polygons.append({"inclusion": False, "polygon": [list(p) for p in z.polygon], "version": 1})

    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": polygons, "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": cruise_speed,
            "firmwareType": 12,                  # MAV_AUTOPILOT_PX4
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": hover_speed,
            "items": items,
            "plannedHomePosition": list(home),
            "vehicleType": 2,                    # MAV_TYPE_QUADROTOR
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def parse_plan(plan: dict) -> dict:
    """Extract home, flyable waypoints and geofence shapes from a .plan dict."""
    if plan.get("fileType") != "Plan":
        raise ValueError("not a QGroundControl plan file (fileType != 'Plan')")
    mission = plan.get("mission", {})
    home = mission.get("plannedHomePosition") or [0, 0, 0]
    if len(home) < 3:
        home = list(home) + [0] * (3 - len(home))

    waypoints = []
    for item in mission.get("items", []):
        if item.get("type") != "SimpleItem":
            continue                              # survey/corridor items are not flown by the sim
        cmd = item.get("command")
        p = item.get("params", [])
        if cmd in NAV_COMMANDS and len(p) >= 7 and p[4] is not None and p[5] is not None:
            lat, lon = float(p[4]), float(p[5])
            alt = float(p[6] if p[6] is not None else item.get("Altitude", 50))
            if lat == 0 and lon == 0:            # takeoff items sometimes carry 0,0
                continue
            waypoints.append({"lat": lat, "lon": lon, "alt": alt, "command": cmd})
        elif cmd == CMD_RTL:
            waypoints.append({"lat": float(home[0]), "lon": float(home[1]),
                              "alt": waypoints[-1]["alt"] if waypoints else 30.0, "command": cmd})

    fence = plan.get("geoFence", {}) or {}
    polygons = [{"inclusion": bool(p.get("inclusion", True)), "polygon": p.get("polygon", [])}
                for p in fence.get("polygons", [])]
    circles = [{"inclusion": bool(c.get("inclusion", True)),
                "center": c.get("circle", {}).get("center"),
                "radius": c.get("circle", {}).get("radius")}
               for c in fence.get("circles", [])]

    return {
        "home": [float(home[0]), float(home[1]), float(home[2] or 0)],
        "waypoints": waypoints,
        "cruise_speed": float(mission.get("cruiseSpeed", 8.0) or 8.0),
        "fence_polygons": polygons,
        "fence_circles": circles,
    }
