# ROS2 Robot — Autonomous Navigation with TurtleBot3

## 📋 Summary

This project implements **autonomous obstacle-avoidance navigation** for a TurtleBot3 robot using **ROS2 Humble** and **Gazebo** simulation. Two navigation strategies are provided and compared:

- **FSM (Finite State Machine)** — a reactive state-based controller that switches between *Forward*, *Avoid*, *Rotate*, and *Stuck Recovery* states based on real-time LiDAR data.
- **Behavior Tree (BT)** — a modular, hierarchical controller built with `py_trees` that handles the same scenarios in a more scalable and readable architecture.

Both approaches share a common LiDAR analysis module (`lidar_zones.py`) that divides the sensor field into zones (front, left, right) and classifies obstacles into danger/avoid/caution levels.

> **Stack:** ROS2 Humble · Python 3.10 · Gazebo 11 · py_trees · Ubuntu 22.04

---

#  — Navigation autonome TurtleBot3
## FSM + Behavior Tree | ROS2 Humble | Ubuntu 22.04

---

## Prérequis système

| Élément | Version |
|---------|---------|
| OS      | Ubuntu 22.04 LTS (Jammy) |
| ROS2    | Humble Hawksbill |
| Python  | 3.10 (inclus dans Ubuntu 22.04) |
| Gazebo  | Gazebo 11 (installé avec Humble) |

---

## ÉTAPE 0 — Installer ROS2 Humble (si pas encore fait)

```bash
# Activer le dépôt ROS2
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop
```

Ajouter au `~/.bashrc` :

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ÉTAPE 1 — Installer les dépendances TurtleBot3 et py_trees

```bash
sudo apt update

# TurtleBot3 pour Gazebo (Humble)
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations

# py_trees pour le Behavior Tree
sudo apt install -y \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces

# Outils de build
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

---

## ÉTAPE 2 — Créer votre workspace et compiler

```bash
# Creer le workspace (une seule fois)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copier le package ici
cp -r /chemin/vers/turtlebot3_nav .

# Revenir a la racine
cd ~/ros2_ws

# Sourcer ROS2 Humble avant de compiler
source /opt/ros/humble/setup.bash

# Compiler uniquement ce package
colcon build --packages-select turtlebot3_nav

# Sourcer le workspace compilé
source install/setup.bash
```

Ajouter le sourcing automatique au `~/.bashrc` :

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ÉTAPE 3 — Configurer le modèle TurtleBot3

```bash
# Ajouter au ~/.bashrc (fait une seule fois)
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

---

## ÉTAPE 4 — Lancer la simulation

### Option A : tout en un (Gazebo + navigation)

```bash
# Version FSM — monde simple
ros2 launch turtlebot3_nav fsm_nav.launch.py

# Version FSM — monde maison (plus complexe)
ros2 launch turtlebot3_nav fsm_nav.launch.py world:=house

# Version BT — monde simple
ros2 launch turtlebot3_nav bt_nav.launch.py

# Version BT — monde maison
ros2 launch turtlebot3_nav bt_nav.launch.py world:=house
```

### Option B : deux terminaux séparés (pour débogage)

**Terminal 1 — Gazebo :**
```bash
source ~/.bashrc
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# ou : ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Terminal 2 — Noeud de navigation :**
```bash
source ~/.bashrc
# FSM :
ros2 run turtlebot3_nav fsm_navigation
# BT  :
ros2 run turtlebot3_nav bt_navigation
```

---

## ÉTAPE 5 — Supervision en temps réel

```bash
# Observer les commandes envoyees au robot
ros2 topic echo /cmd_vel

# Observer les mesures Lidar (brutes)
ros2 topic echo /scan --no-arr    # sans les tableaux pour la lisibilite

# Lister tous les noeuds actifs
ros2 node list

# Voir les topics actifs
ros2 topic list

# Info sur le noeud FSM
ros2 node info /fsm_navigation

# Log niveau DEBUG pour le BT
ros2 run turtlebot3_nav bt_navigation --ros-args --log-level debug
```

---

## Structure du package

```
turtlebot3_nav/
│
├── package.xml                  Configuration ROS2 du package
├── setup.py                     Build Python / points d'entrée
├── setup.cfg                    Config ament_python
├── README.md                    Ce fichier
├── RAPPORT.md                   Rapport technique
│
├── resource/
│   └── turtlebot3_nav        Marker ament_index (ne pas supprimer)
│
├── launch/
│   ├── fsm_nav.launch.py        Lancement FSM + Gazebo
│   └── bt_nav.launch.py         Lancement BT  + Gazebo
│
└── turtlebot3_nav/
    ├── __init__.py
    ├── lidar_zones.py            Module partage : analyse multi-zones Lidar
    ├── fsm_navigation.py         Version 1 : Machine a Etats Finis
    └── bt_navigation.py          Version 2 : Behavior Tree
```

---

## Paramètres ajustables

Tous les paramètres sont regroupés en haut de chaque fichier Python.

| Paramètre | Fichier | Valeur par défaut | Description |
|-----------|---------|-------------------|-------------|
| `DIST_DANGER`  | lidar_zones.py | 0.25 m | Seuil arrêt d'urgence |
| `DIST_AVOID`   | lidar_zones.py | 0.50 m | Seuil évitement |
| `DIST_CAUTION` | lidar_zones.py | 0.80 m | Seuil ralentissement |
| `LINEAR_MAX`   | fsm / bt       | 0.22 m/s | Vitesse max |
| `ANGULAR_FAST` | fsm / bt       | 1.40 rad/s | Rotation rapide |
| `OSC_THRESHOLD`| fsm_navigation | 4 | Seuil anti-oscillation |
| `STUCK_TIME_S` | fsm_navigation | 3.0 s | Délai détection blocage |

---

## Dépannage courant

| Problème | Solution |
|----------|----------|
| `Package 'turtlebot3_nav' not found` | `source ~/ros2_ws/install/setup.bash` |
| `TURTLEBOT3_MODEL not set` | `export TURTLEBOT3_MODEL=waffle` |
| Gazebo ne démarre pas | `source /opt/ros/humble/setup.bash` dans le terminal |
| `ModuleNotFoundError: py_trees` | `sudo apt install ros-humble-py-trees-ros` |
| Robot ne bouge pas | Vérifier `/scan` : `ros2 topic hz /scan` |
| Le robot oscille | Réduire `OSC_THRESHOLD` ou augmenter `DIST_AVOID` |
