# Rapport Technique — TP5
## Navigation autonome avec évitement d'obstacles
### TurtleBot3 Waffle | ROS2 Humble | Ubuntu 22.04

---

## 1. Objectif

Implémenter deux architectures de navigation réactive permettant au robot TurtleBot3 Waffle de naviguer de façon autonome dans un environnement inconnu en détectant et évitant les obstacles à l'aide du capteur Lidar 360°.

---

## 2. Module commun : `lidar_zones.py`

### 2.1 Motivation

L'approche naïve (seuil global sur les 360 lectures brutes) ne permet pas de savoir si l'obstacle se trouve à gauche, à droite ou directement devant. L'analyse multi-zones résout ce problème.

### 2.2 Découpage en 6 zones

```
               FRONT_LEFT   FRONT   FRONT_RIGHT
                 [30-90°]  [-30/+30°]  [270-330°]
                    ╲         |         ╱
          LEFT       ╲       [R]       ╱   RIGHT
        [90-135°]     ╲     /   \     ╱  [225-270°]
                       ─────────────────
                        REAR [135-225°]
```

### 2.3 Seuils de distance

| Seuil | Valeur | Effet déclenché |
|-------|--------|-----------------|
| DIST_DANGER  | 0.25 m | Arrêt d'urgence (STOP / SafetyCheck) |
| DIST_AVOID   | 0.50 m | Déclenchement de l'évitement |
| DIST_CAUTION | 0.80 m | Début du ralentissement progressif |
| DIST_CLEAR   | 1.20 m | Zone considérée libre |

### 2.4 Filtrage des mesures

- Rejet des valeurs `NaN` et `Inf` (réflexions parasites Gazebo).
- Rejet des distances hors bornes physiques `[range_min, range_max]`.
- Calcul de la distance minimale (pour les décisions de sécurité).
- Moyenne trimée à 10 % (pour la distance moyenne, robuste aux outliers).
- Direction de l'obstacle : moyenne pondérée par `1/distance` (les obstacles proches influencent plus la direction).

---

## 3. Version 1 — Machine à États Finis (FSM)

### 3.1 Graphe des états et transitions

```
                 ┌──────────────────┐
     ┌──────────►│     FORWARD      │◄──────────────────────┐
     │           │ vitesse adaptive │                       │
     │           └────────┬─────────┘                       │ récupéré
     │             gauche │ droite                          │ (tour_history.clear)
     │             libre  │ libre                           │
     │         ┌──────────┴──────────────┐                  │
     │         ▼                         ▼                  │
     │  ┌─────────────┐         ┌──────────────┐           │
     │  │ AVOID_LEFT  │         │ AVOID_RIGHT  │           │
     │  │ tourne ←    │         │   tourne →   │           │
     │  └──────┬───── ┘         └───────┬──────┘           │
     │         │                        │                   │
     │         └──────────┬─────────────┘                  │
     │             oscillation détectée                     │
     │          (ou danger toutes zones bloquées)            │
     │                    ▼                                 │
     │           ┌────────────────┐                        │
     └───────────│    RECOVER     │────────────────────────┘
                 │ recul + pivot  │
                 └───────┬────────┘
                         │ blocage persistant
                         ▼
                  ┌────────────┐
                  │    STOP    │◄── danger immédiat (toute zone < 0.25 m)
                  └────────────┘
                         │ danger disparu
                         └──────────────► FORWARD
```

### 3.2 Description des états

**FORWARD** : avance à vitesse calculée par interpolation linéaire entre `LINEAR_MIN` et `LINEAR_MAX` selon le facteur `speed_factor` du module lidar_zones. Dès qu'une zone avant est bloquée, `best_escape_side` compare les distances latérales pour choisir le meilleur côté.

**AVOID_LEFT / AVOID_RIGHT** : le robot pivote avec une vitesse angulaire lente (obstacle lointain) ou rapide (obstacle proche). Une légère composante linéaire est conservée si l'obstacle est à plus de 35 cm. Chaque changement de direction est enregistré dans l'historique glissant.

**RECOVER** : séquence en deux phases (recul 1 s, puis rotation 1,5 s). L'historique des virages est effacé à la fin.

**STOP** : arrêt total déclenché quand un obstacle passe sous `DIST_DANGER = 0.25 m`. Le robot reprend dès la disparition du danger.

### 3.3 Améliorations

#### Anti-oscillation par historique glissant

Un `deque` de taille `OSC_WINDOW = 6` mémorise les dernières directions de virage. Si le nombre d'alternances gauche/droite dépasse `OSC_THRESHOLD = 4`, le robot bascule en RECOVER. Ce mécanisme évite le piège classique de deux murs parallèles qui font alterner indéfiniment AVOID_LEFT et AVOID_RIGHT.

#### Détection de blocage par odométrie

Toutes les `STUCK_CHECK_PERIOD = 3` secondes, le déplacement cumulé depuis la dernière vérification est calculé via l'odométrie (`/odom`). Si le robot est censé avancer (état FORWARD) mais n'a parcouru que moins de `STUCK_MIN_DIST_M = 0.05 m`, il est considéré bloqué et passe en RECOVER.

---

## 4. Version 2 — Behavior Tree (BT)

### 4.1 Arbre complet

```
Root [Sequence]  ──────────────────────────────────────────────────────
 │
 ├─ SafetyCheck (Condition)
 │    FAILURE + stop si any_danger
 │    SUCCESS  sinon
 │
 └─ Navigate [Selector]
      │
      ├─ AvoidObstacle [Sequence]
      │   │
      │   ├─ ObstacleAhead? (Condition)
      │   │    SUCCESS si front/front_left/front_right bloqué
      │   │
      │   └─ ChooseSide [Selector]
      │        │
      │        ├─ GoLeft [Sequence]
      │        │   ├─ LeftIsClear? (Condition)  front_left + left libres
      │        │   └─ TurnLeft     (Action)      RUNNING → SUCCESS voie libre
      │        │
      │        ├─ GoRight [Sequence]
      │        │   ├─ RightIsClear? (Condition)  front_right + right libres
      │        │   └─ TurnRight     (Action)      RUNNING → SUCCESS voie libre
      │        │
      │        └─ Recover (Action)
      │             Phase 0 : recul  1.0 s
      │             Phase 1 : rotation 1.5 s
      │             → SUCCESS
      │
      └─ MoveForward (Action)  vitesse adaptative, toujours RUNNING
```

### 4.2 Tableau des statuts

| Noeud | Type | SUCCESS | FAILURE | RUNNING |
|-------|------|---------|---------|---------|
| SafetyCheck   | Condition | pas de danger | danger < 0.25 m (+ stop) | scan absent |
| ObstacleAhead | Condition | obstacle détecté | voie libre | — |
| LeftIsClear   | Condition | gauche libre | gauche bloqué | — |
| RightIsClear  | Condition | droite libre | droite bloqué | — |
| TurnLeft      | Action | avant dégagé | — | rotation en cours |
| TurnRight     | Action | avant dégagé | — | rotation en cours |
| Recover       | Action | fin de séquence | — | recul/rotation |
| MoveForward   | Action | — (jamais) | — | toujours |

### 4.3 Blackboard

Toutes les feuilles lisent le Lidar via le Blackboard py_trees (clé `'lidar'`). Le noeud ROS2 est le seul écrivain. Les lecteurs créent leur propre `Client` en lecture seule, garantissant l'isolation des accès.

---

## 5. Comparaison FSM vs BT

| Critère | FSM | BT |
|---------|-----|----|
| Lisibilité | Bonne pour ≤ 10 états | Excellente, arbre visuel |
| Ajout de comportements | O(n) transitions à modifier | Ajout d'une feuille |
| Débogage | Logs d'état suffisants | Visualiseur py_trees disponible |
| Réactivité | Très haute | Haute (tick 10 Hz) |
| Gestion des priorités | Explicite dans le code | Naturelle (Sequence/Selector) |
| Composition | Difficile | Naturelle (sous-arbres) |

---

## 6. Difficultés rencontrées

**Oscillation entre deux obstacles parallèles** : résolue par l'historique des virages (FSM) et la structure du Selector (BT tente GoLeft puis GoRight dans l'ordre, mais finit par Recover si les deux échouent).

**Angles morts du Lidar** : le TurtleBot3 Waffle a un Lidar 360° sans angle mort physique. Les lectures `Inf` issues des réflexions sur les surfaces blanches de Gazebo sont filtrées par le test `math.isinf()` dans `analyze_scan`.

**Blocage dans un coin concave** : la phase de recul dans RECOVER crée suffisamment d'espace pour que la rotation réoriente le robot.

**API py_trees avec Humble** : la version 2.x de py_trees requiert d'appeler `enable_activity_stream()` avant de créer les Clients du Blackboard. Les Clients partagent les clés avec des permissions explicites (READ / WRITE). La méthode `initialise()` d'une Action est appelée automatiquement à chaque réactivation du noeud.

---

## 7. Conclusion

Les deux architectures répondent à l'objectif de navigation réactive dans `turtlebot3_world` et `turtlebot3_house`. Le module commun `lidar_zones.py` assure la cohérence des deux implémentations. La FSM est plus simple à déboguer pour un comportement limité ; le BT est plus extensible pour des comportements plus complexes (navigation vers un but, gestion de la batterie, etc.).
