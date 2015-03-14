ot-robo
=======

# SLAM

Après un bringup, lancer :

- rplidar : `roslaunch rplidar_ros rplidar.launch`
- mapping : `roslaunch hector_mapping mapping_default.launch`
- planification : `rosrun slam_strategy planification`
- mouvement : `rosrun slam_strategy motion`

N.B. : le paquet `slam_strategy` risque de changer de nom.

# Communication multi-robots

1. Lancer `master_discovery` sur chaque machine.
2. Lancer `master_sync` sur les machines qui synchronisent (eg. toutes) avec
   `_sync_topics:=['liste_des', 'topic_a', 'synchroniser']`.
3. It works.

Notes supplémentaires :

- `roslaunch master_sync_fkie master_sync _sync_topics=['broadcastMap']`
- Package `merge_map`
- Listener map forward la map de `hector_mapping` (topic /map) sur un topic /broadcastMap
- `listener_broadcast` écoute ce topic et affiche le header.
- À tester en réseau
