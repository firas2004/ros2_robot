"""
lidar_zones.py — Module partagé d'analyse multi-zones du Lidar
Compatible ROS2 Humble (Ubuntu 22.04)

Zones analysées (vue de dessus, sens trigonométrique) :
  FRONT       : ±30°  autour de 0°
  FRONT_LEFT  : 30°–90°
  FRONT_RIGHT : 270°–330°
  LEFT        : 90°–150°
  RIGHT       : 210°–270°
  REAR        : 150°–210°
"""

import math
from sensor_msgs.msg import LaserScan

# ── Distances seuils ────────────────────────────────────────────────────────
DIST_AVOID   = 0.45   # [m] déclenchement évitement
DIST_WARN    = 0.70   # [m] avertissement précoce (évitement progressif)
DIST_CLEAR   = 0.90   # [m] zone considérée libre
DIST_SIDE    = 0.30   # [m] seuil latéral (évitement de mur serré)

# ── Angles des zones (degrés) ────────────────────────────────────────────────
ZONE_DEFS = {
    "front":       (-30,  30),
    "front_left":  ( 30,  90),
    "front_right": (-90, -30),
    "left":        ( 90, 150),
    "right":       (-150, -90),
    "rear":        (150, 180),   # + (-180,-150) géré dans get_zone_min
}


def _angle_indices(scan: LaserScan, deg_min: float, deg_max: float):
    """Retourne les indices du scan correspondant à [deg_min, deg_max]."""
    rad_min = math.radians(deg_min)
    rad_max = math.radians(deg_max)
    n = len(scan.ranges)
    indices = []
    for i in range(n):
        angle = scan.angle_min + i * scan.angle_increment
        if rad_min <= angle <= rad_max:
            indices.append(i)
    return indices


def get_zone_min(scan: LaserScan, zone: str) -> float:
    """
    Retourne la distance minimale valide dans la zone demandée.
    Retourne float('inf') si aucune mesure valide.
    """
    if zone == "rear":
        # Zone arrière : 150°–180° ET −180°–−150°
        d1 = _get_range_min(scan, 150, 180)
        d2 = _get_range_min(scan, -180, -150)
        return min(d1, d2)
    lo, hi = ZONE_DEFS[zone]
    return _get_range_min(scan, lo, hi)


def _get_range_min(scan: LaserScan, deg_min: float, deg_max: float) -> float:
    indices = _angle_indices(scan, deg_min, deg_max)
    if not indices:
        return float('inf')
    valid = [
        scan.ranges[i]
        for i in indices
        if scan.range_min < scan.ranges[i] < scan.range_max
        and not math.isnan(scan.ranges[i])
        and not math.isinf(scan.ranges[i])
    ]
    return min(valid) if valid else float('inf')


def analyze_scan(scan: LaserScan) -> dict:
    """
    Analyse complète du scan et retourne un dict avec :
      - distances minimales par zone
      - flags booléens obstacle/avertissement
      - direction recommandée d'évitement ('left', 'right', 'back')
    """
    zones = {z: get_zone_min(scan, z) for z in ZONE_DEFS}

    result = {
        "distances": zones,

        # Flags obstacle (seuil dur)
        "obstacle_front":       zones["front"]       < DIST_AVOID,
        "obstacle_front_left":  zones["front_left"]  < DIST_AVOID,
        "obstacle_front_right": zones["front_right"] < DIST_AVOID,
        "obstacle_left":        zones["left"]         < DIST_SIDE,
        "obstacle_right":       zones["right"]        < DIST_SIDE,

        # Avertissements précoces (évitement progressif)
        "warn_front":      zones["front"]      < DIST_WARN,
        "warn_front_left": zones["front_left"] < DIST_WARN,
        "warn_front_right":zones["front_right"]< DIST_WARN,

        # Espace libre global devant
        "front_clear": zones["front"] > DIST_CLEAR,
    }

    # Direction recommandée d'évitement
    if result["obstacle_front"]:
        # Choisir le côté avec le plus d'espace
        if zones["front_left"] >= zones["front_right"]:
            result["avoid_direction"] = "left"
        else:
            result["avoid_direction"] = "right"
        # Si les deux côtés sont bouchés, reculer
        if zones["front_left"] < DIST_AVOID and zones["front_right"] < DIST_AVOID:
            result["avoid_direction"] = "back"
    else:
        result["avoid_direction"] = "none"

    # Facteur de réduction de vitesse linéaire (évitement progressif)
    # 1.0 = pleine vitesse, 0.0 = arrêt
    if zones["front"] < DIST_AVOID:
        result["speed_factor"] = 0.0
    elif zones["front"] < DIST_WARN:
        result["speed_factor"] = (zones["front"] - DIST_AVOID) / (DIST_WARN - DIST_AVOID)
    else:
        result["speed_factor"] = 1.0

    return result
