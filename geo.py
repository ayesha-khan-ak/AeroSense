"""
AeroSense — geo.py
Small, dependency-free geodesy helpers used by the geofence engine,
the virtual drone and the radar/LiDAR adapters.

Coordinate systems used in the project
--------------------------------------
* WGS84 geodetic      : (lat, lon) in degrees, altitude in metres.
* Local ENU (metres)  : East, North, Up relative to an anchor point
                        (the sensor site on FAST campus).  Radar / LiDAR
                        detections arrive in a sensor-relative frame and
                        are converted to ENU, then to lat/lon.
* Altitude conventions: "alt_rel" = metres above the home / ground level
                        (AGL, what geofence altitude bands use);
                        "alt_msl" = metres above mean sea level.

For distances of a few kilometres (a campus) a local tangent-plane
approximation is accurate to centimetres, so we don't need pyproj.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

# WGS84 ellipsoid
_A = 6378137.0                 # semi-major axis (m)
_F = 1 / 298.257223563         # flattening
_E2 = _F * (2 - _F)            # first eccentricity squared


def _radii(lat_deg: float) -> tuple[float, float]:
    """Meridian (north-south) and prime-vertical (east-west) radii of curvature."""
    s = math.sin(math.radians(lat_deg))
    d = 1 - _E2 * s * s
    r_n = _A / math.sqrt(d)                 # prime vertical
    r_m = _A * (1 - _E2) / (d ** 1.5)       # meridian
    return r_m, r_n


def geodetic_to_enu(lat: float, lon: float, alt: float,
                    lat0: float, lon0: float, alt0: float = 0.0) -> tuple[float, float, float]:
    """WGS84 point -> local East/North/Up metres around anchor (lat0, lon0, alt0)."""
    r_m, r_n = _radii(lat0)
    north = math.radians(lat - lat0) * r_m
    east = math.radians(lon - lon0) * r_n * math.cos(math.radians(lat0))
    return east, north, alt - alt0


def enu_to_geodetic(east: float, north: float, up: float,
                    lat0: float, lon0: float, alt0: float = 0.0) -> tuple[float, float, float]:
    """Local East/North/Up metres around the anchor -> WGS84 (lat, lon, alt)."""
    r_m, r_n = _radii(lat0)
    lat = lat0 + math.degrees(north / r_m)
    lon = lon0 + math.degrees(east / (r_n * math.cos(math.radians(lat0))))
    return lat, lon, alt0 + up


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle ground distance in metres."""
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2, degrees clockwise from true north."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def enu_to_polar(east: float, north: float, up: float) -> tuple[float, float, float]:
    """ENU -> (slant range m, azimuth deg from north clockwise, elevation deg)."""
    ground = math.hypot(east, north)
    rng = math.sqrt(ground * ground + up * up)
    az = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
    el = math.degrees(math.atan2(up, ground)) if rng > 0 else 0.0
    return rng, az, el


def polar_to_enu(range_m: float, azimuth_deg: float, elevation_deg: float) -> tuple[float, float, float]:
    """(slant range, azimuth from north clockwise, elevation) -> ENU metres."""
    el, az = math.radians(elevation_deg), math.radians(azimuth_deg)
    ground = range_m * math.cos(el)
    return ground * math.sin(az), ground * math.cos(az), range_m * math.sin(el)


def sensor_xyz_to_enu(x: float, y: float, z: float, heading_deg: float) -> tuple[float, float, float]:
    """
    Sensor body frame (x = forward, y = left, z = up — the usual LiDAR/ROS
    convention, also used by MMAUD) -> ENU, given the compass bearing the
    sensor's x-axis points to.
    """
    h = math.radians(heading_deg)
    east = x * math.sin(h) - y * math.cos(h)
    north = x * math.cos(h) + y * math.sin(h)
    return east, north, z


# ---------------------------------------------------------------- polygons --

LatLon = Sequence[float]


def _to_local(points: Iterable[LatLon], lat0: float, lon0: float) -> list[tuple[float, float]]:
    out = []
    for p in points:
        e, n, _ = geodetic_to_enu(p[0], p[1], 0.0, lat0, lon0)
        out.append((e, n))
    return out


def polygon_centroid(poly: Sequence[LatLon]) -> tuple[float, float]:
    lat = sum(p[0] for p in poly) / len(poly)
    lon = sum(p[1] for p in poly) / len(poly)
    return lat, lon


def point_in_polygon(lat: float, lon: float, poly: Sequence[LatLon]) -> bool:
    """Ray-casting test (works for convex and concave polygons)."""
    if len(poly) < 3:
        return False
    pts = _to_local(poly, lat, lon)      # query point becomes the origin
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > 0) != (yj > 0):
            x_cross = xi + (0 - yi) * (xj - xi) / (yj - yi)
            if x_cross > 0:
                inside = not inside
        j = i
    return inside


def distance_to_polygon_edge_m(lat: float, lon: float, poly: Sequence[LatLon]) -> float:
    """Shortest horizontal distance (m) from the point to the polygon boundary."""
    pts = _to_local(poly, lat, lon)
    best = float("inf")
    for i in range(len(pts)):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % len(pts)]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / seg2))
        px, py = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(px, py))
    return best


def signed_distance_m(lat: float, lon: float, poly: Sequence[LatLon]) -> float:
    """Negative inside the polygon, positive outside (metres to the boundary)."""
    d = distance_to_polygon_edge_m(lat, lon, poly)
    return -d if point_in_polygon(lat, lon, poly) else d


def polygon_area_m2(poly: Sequence[LatLon]) -> float:
    lat0, lon0 = polygon_centroid(poly)
    pts = _to_local(poly, lat0, lon0)
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0
