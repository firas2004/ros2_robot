#!/usr/bin/env python3
"""
fsm_navigation.py — Version 1 : Machine à États Finis (FSM)
Compatible ROS2 Humble (Ubuntu 22.04)

États :
  FORWARD   → avance en ligne droite (vitesse progressive selon distance)
  TURN_LEFT → tourne à gauche pour éviter un obstacle
  TURN_RIGHT→ tourne à droite pour éviter un obstacle
  BACK_UP   → recule quand les deux côtés sont bouchés
  STUCK     → manœuvre spéciale anti-blocage (rotation prolongée)

Améliorations :
  - Évitement progressif (speed_factor)
  - Anti-oscillation (compteur de changements d'état)
  - Détection de blocage (STUCK_TIME_S sans avancer)
  - Analyse multi-zones Lidar via lidar_zones.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math

from turtlebot3_nav.lidar_zones import analyze_scan

# ── Vitesses ─────────────────────────────────────────────────────────────────
LINEAR_MAX    = 0.22   # m/s  vitesse max en ligne droite
LINEAR_BACK   = 0.10   # m/s  vitesse de recul
ANGULAR_SLOW  = 0.5    # rad/s rotation douce
ANGULAR_FAST  = 1.5    # rad/s rotation rapide (évitement urgent)
ANGULAR_STUCK = 2.0    # rad/s rotation anti-blocage

# ── Paramètres anti-oscillation ───────────────────────────────────────────────
OSC_WINDOW    = 10     # nombre de transitions récentes à surveiller
OSC_THRESHOLD = 6      # si > OSC_THRESHOLD alternances dans la fenêtre → oscillation

# ── Paramètre anti-blocage ────────────────────────────────────────────────────
STUCK_TIME_S  = 3.0    # secondes sans avancer → état STUCK
STUCK_DURATION= 2.0    # durée de la manœuvre anti-blocage

# ── États ─────────────────────────────────────────────────────────────────────
STATE_FORWARD      = "FORWARD"
STATE_TURN_LEFT    = "TURN_LEFT"
STATE_TURN_RIGHT   = "TURN_RIGHT"
STATE_BACK_UP      = "BACK_UP"
STATE_STUCK        = "STUCK"


class FSMNavigation(Node):
    def __init__(self):
        super().__init__('fsm_navigation')
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan',
                                                  self.scan_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.state       = STATE_FORWARD
        self.lidar_data  = None
        self.analysis    = None

        # Anti-oscillation
        self._state_history = []   # historique des états récents

        # Anti-blocage
        self._last_forward_time = time.time()
        self._stuck_start       = None
        self._stuck_direction   = 1.0  # +1 gauche, −1 droite

        self.get_logger().info("FSM Navigation démarrée (ROS2 Humble)")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        self.lidar_data = msg
        self.analysis   = analyze_scan(msg)

    # ── Boucle principale ─────────────────────────────────────────────────────
    def control_loop(self):
        if self.analysis is None:
            return

        a = self.analysis
        new_state = self._compute_next_state(a)

        # Anti-oscillation : si on alterne trop vite entre TURN_LEFT et TURN_RIGHT
        if new_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT):
            self._state_history.append(new_state)
            if len(self._state_history) > OSC_WINDOW:
                self._state_history.pop(0)
            if self._is_oscillating():
                # Forcer une direction et l'ignorer pendant quelques cycles
                new_state = STATE_TURN_LEFT  # direction arbitraire pour casser l'oscillation
                self.get_logger().warn("Oscillation détectée → forçage gauche")
        else:
            self._state_history.clear()

        # Détection blocage
        if new_state == STATE_FORWARD:
            self._last_forward_time = time.time()
        elif new_state in (STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_BACK_UP):
            elapsed = time.time() - self._last_forward_time
            if elapsed > STUCK_TIME_S and self._stuck_start is None:
                self._stuck_start     = time.time()
                self._stuck_direction = 1.0 if a["avoid_direction"] != "left" else -1.0
                new_state = STATE_STUCK
                self.get_logger().warn("Robot bloqué → manœuvre STUCK")

        # Sortie de l'état STUCK
        if self.state == STATE_STUCK:
            if self._stuck_start and (time.time() - self._stuck_start) > STUCK_DURATION:
                self._stuck_start           = None
                self._last_forward_time     = time.time()
                new_state = STATE_FORWARD
                self.get_logger().info("Fin manœuvre STUCK → FORWARD")
            else:
                new_state = STATE_STUCK  # rester dans STUCK

        if new_state != self.state:
            self.get_logger().info(f"État : {self.state} → {new_state}")
        self.state = new_state

        self._execute_state(a)

    def _compute_next_state(self, a: dict) -> str:
        if a["avoid_direction"] == "back":
            return STATE_BACK_UP
        if a["obstacle_front"]:
            if a["avoid_direction"] == "left":
                return STATE_TURN_LEFT
            return STATE_TURN_RIGHT
        # Avertissement précoce → tourner doucement
        if a["warn_front"] and not a["warn_front_left"] and a["warn_front_right"]:
            return STATE_TURN_LEFT
        if a["warn_front"] and a["warn_front_left"] and not a["warn_front_right"]:
            return STATE_TURN_RIGHT
        return STATE_FORWARD

    def _is_oscillating(self) -> bool:
        if len(self._state_history) < OSC_WINDOW:
            return False
        alternances = sum(
            1 for i in range(1, len(self._state_history))
            if self._state_history[i] != self._state_history[i - 1]
        )
        return alternances > OSC_THRESHOLD

    def _execute_state(self, a: dict):
        cmd = Twist()
        if self.state == STATE_FORWARD:
            cmd.linear.x  = LINEAR_MAX * a["speed_factor"]
            cmd.angular.z = 0.0

        elif self.state == STATE_TURN_LEFT:
            cmd.linear.x  = 0.0
            # Rotation rapide si obstacle proche, douce si avertissement seulement
            cmd.angular.z = ANGULAR_FAST if a["obstacle_front"] else ANGULAR_SLOW

        elif self.state == STATE_TURN_RIGHT:
            cmd.linear.x  = 0.0
            cmd.angular.z = -(ANGULAR_FAST if a["obstacle_front"] else ANGULAR_SLOW)

        elif self.state == STATE_BACK_UP:
            cmd.linear.x  = -LINEAR_BACK
            cmd.angular.z = 0.0

        elif self.state == STATE_STUCK:
            cmd.linear.x  = -LINEAR_BACK * 0.5
            cmd.angular.z = self._stuck_direction * ANGULAR_STUCK

        self.pub_cmd.publish(cmd)

    def destroy_node(self):
        # Arrêt propre du robot
        self.pub_cmd.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FSMNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
