Lancer un master_discovery sur chaque ordi 
roslaunch master_discovery_fkie master_discovery ?
lancer un master_sync sur les ordi qui sync (tous) avec _sync_topics:=['topic a sync']
roslaunch master_sync_fkie master_sync _sync_topics=['broadcastMap']
package merge_map
listener map rebroadcast la map de hector mapping (/map) sur un topic /broadcastMap
listener_broadcast écoute se topic et affiche le header
à tester en réseau
