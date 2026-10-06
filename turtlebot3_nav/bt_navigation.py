#!/usr/bin/env python3
"""
bt_navigation.py — Version 2 : Behavior Tree (BT)
Compatible ROS2 Humble (Ubuntu 22.04) + py_trees 2.x

Arbre de comportement :
  Selector (priorité)
  ├── Sequence STUCK_RECOVERY      ← si bloqué depuis trop longtemps
  │     ├── IsStuck?
  │     └── BackupAndSpin
  ├── Sequence OBSTACLE_AVOIDANCE  ← si obstacle devant
  │     ├── IsObstacleAhead?
  │     └── TurnAway
  ├── Sequence WARN_SLOW           ← si obstacle proche (avertissement)
  │     ├── IsWarning?
  │     └── SlowDown
  └── Action DRIVE_FORWARD         ← sinon avancer normalement

Améliorations :
  - Évitement progressif via speed_factor
  - Détection de blocage dans le Blackboard
  - Analyse multi-zones Lidar
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import py_trees

from turtlebot3_nav.lidar_zones import analyze_scan

# ── Vitesses ─────────────────────────────────────────────────────────────────
LINEAR_MAX   = 0.22
LINEAR_BACK  = 0.10
ANGULAR_FAST = 1.5
ANGULAR_SLOW = 0.5
ANGULAR_STUCK= 2.0

# ── Anti-blocage ──────────────────────────────────────────────────────────────
STUCK_TIME_S  = 3.0
STUCK_DURATION= 2.0


# ═══════════════════════════════════════════════════════════════════════════════
# Comportements (feuilles de l'arbre)
# ═══════════════════════════════════════════════════════════════════════════════

class IsStuck(py_trees.behaviour.Behaviour):
    """Condition : le robot est-il bloqué depuis STUCK_TIME_S ?"""
    def __init__(self, blackboard):
        super().__init__("IsStuck?")
        self.bb = blackboard

    def update(self):
        data = self.bb.get("lidar_analysis")
        stuck_since = self.bb.get("stuck_since")
        if data is None:
            return py_trees.common.Status.FAILURE
        # Si on n'avance pas et que le chrono est actif
        if not data["front_clear"] and stuck_since is not None:
            elapsed = time.time() - stuck_since
            if elapsed > STUCK_TIME_S:
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BackupAndSpin(py_trees.behaviour.Behaviour):
    """Action : reculer + pivoter pour se désengager."""
    def __init__(self, blackboard, pub):
        super().__init__("BackupAndSpin")
        self.bb  = blackboard
        self.pub = pub
        self._start = None

    def initialise(self):
        self._start = time.time()
        data = self.bb.get("lidar_analysis") or {}
        avoid = data.get("avoid_direction", "left")
        self._dir = 1.0 if avoid != "right" else -1.0

    def update(self):
        if (time.time() - self._start) < STUCK_DURATION:
            cmd = Twist()
            cmd.linear.x  = -LINEAR_BACK * 0.5
            cmd.angular.z = self._dir * ANGULAR_STUCK
            self.pub.publish(cmd)
            return py_trees.common.Status.RUNNING
        # Manœuvre terminée → réinitialiser le chrono
        self.bb.set("stuck_since", None)
        return py_trees.common.Status.SUCCESS


class IsObstacleAhead(py_trees.behaviour.Behaviour):
    """Condition : obstacle dans la zone frontale (seuil dur)."""
    def __init__(self, blackboard):
        super().__init__("IsObstacleAhead?")
        self.bb = blackboard

    def update(self):
        data = self.bb.get("lidar_analysis")
        if data and data["obstacle_front"]:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class TurnAway(py_trees.behaviour.Behaviour):
    """Action : tourner du côté le plus dégagé."""
    def __init__(self, blackboard, pub):
        super().__init__("TurnAway")
        self.bb  = blackboard
        self.pub = pub

    def update(self):
        data = self.bb.get("lidar_analysis")
        if data is None:
            return py_trees.common.Status.FAILURE
        cmd = Twist()
        cmd.linear.x = 0.0
        if data["avoid_direction"] == "back":
            cmd.linear.x  = -LINEAR_BACK
            cmd.angular.z = 0.0
        elif data["avoid_direction"] == "right":
            cmd.angular.z = -ANGULAR_FAST
        else:
            cmd.angular.z = ANGULAR_FAST
        self.pub.publish(cmd)
        return py_trees.common.Status.SUCCESS


class IsWarning(py_trees.behaviour.Behaviour):
    """Condition : obstacle en zone d'avertissement (seuil doux)."""
    def __init__(self, blackboard):
        super().__init__("IsWarning?")
        self.bb = blackboard

    def update(self):
        data = self.bb.get("lidar_analysis")
        if data and data["warn_front"] and not data["obstacle_front"]:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class SlowDown(py_trees.behaviour.Behaviour):
    """Action : avancer lentement avec légère correction angulaire."""
    def __init__(self, blackboard, pub):
        super().__init__("SlowDown")
        self.bb  = blackboard
        self.pub = pub

    def update(self):
        data = self.bb.get("lidar_analysis")
        if data is None:
            return py_trees.common.Status.FAILURE
        cmd = Twist()
        cmd.linear.x = LINEAR_MAX * data["speed_factor"]
        # Correction douce vers l'espace libre
        if data["warn_front_right"] and not data["warn_front_left"]:
            cmd.angular.z = ANGULAR_SLOW
        elif data["warn_front_left"] and not data["warn_front_right"]:
            cmd.angular.z = -ANGULAR_SLOW
        else:
            cmd.angular.z = 0.0
        self.pub.publish(cmd)
        return py_trees.common.Status.SUCCESS


class DriveForward(py_trees.behaviour.Behaviour):
    """Action : avancer en ligne droite à pleine vitesse."""
    def __init__(self, blackboard, pub):
        super().__init__("DriveForward")
        self.bb  = blackboard
        self.pub = pub

    def update(self):
        cmd = Twist()
        cmd.linear.x  = LINEAR_MAX
        cmd.angular.z = 0.0
        self.pub.publish(cmd)
        return py_trees.common.Status.SUCCESS


# ═══════════════════════════════════════════════════════════════════════════════
# Nœud ROS2
# ═══════════════════════════════════════════════════════════════════════════════

class BTNavigation(Node):
    def __init__(self):
        super().__init__('bt_navigation')
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan',
                                                  self.scan_callback, 10)
        self.timer    = self.create_timer(0.1, self.tick)

        # Blackboard py_trees 2.x
        self.bb = py_trees.blackboard.Client(name="BTNav")
        self.bb.register_key("lidar_analysis", access=py_trees.common.Access.WRITE)
        self.bb.register_key("stuck_since",    access=py_trees.common.Access.WRITE)
        self.bb.lidar_analysis = None
        self.bb.stuck_since    = None

        self._last_forward_time = time.time()
        self._tree = self._build_tree()
        self._tree.setup(timeout=10)

        self.get_logger().info("BT Navigation démarrée (ROS2 Humble)")

    def _build_tree(self):
        bb = self.bb
        pub = self.pub_cmd

        # ── Séquence STUCK RECOVERY ──────────────────────────────────────────
        stuck_seq = py_trees.composites.Sequence(
            name="STUCK_RECOVERY", memory=True
        )
        stuck_seq.add_children([
            IsStuck(bb),
            BackupAndSpin(bb, pub),
        ])

        # ── Séquence OBSTACLE AVOIDANCE ──────────────────────────────────────
        avoid_seq = py_trees.composites.Sequence(
            name="OBSTACLE_AVOIDANCE", memory=True
        )
        avoid_seq.add_children([
            IsObstacleAhead(bb),
            TurnAway(bb, pub),
        ])

        # ── Séquence WARN SLOW ───────────────────────────────────────────────
        warn_seq = py_trees.composites.Sequence(
            name="WARN_SLOW", memory=True
        )
        warn_seq.add_children([
            IsWarning(bb),
            SlowDown(bb, pub),
        ])

        # ── Sélecteur racine ─────────────────────────────────────────────────
        root = py_trees.composites.Selector(name="ROOT", memory=False)
        root.add_children([
            stuck_seq,
            avoid_seq,
            warn_seq,
            DriveForward(bb, pub),
        ])

        return py_trees.trees.BehaviourTree(root)

    def scan_callback(self, msg: LaserScan):
        analysis = analyze_scan(msg)
        self.bb.lidar_analysis = analysis

        # Mise à jour du chrono anti-blocage
        if analysis["front_clear"]:
            self._last_forward_time = time.time()
            self.bb.stuck_since = None
        else:
            if self.bb.stuck_since is None:
                self.bb.stuck_since = self._last_forward_time

    def tick(self):
        if self.bb.lidar_analysis is None:
            return
        self._tree.tick()

    def destroy_node(self):
        self.pub_cmd.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BTNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
