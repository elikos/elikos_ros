# ELIKOS_MOVEIT_CONFIG

## Buts  
* Package en partie auto-généré par MoveitSetupAssistant. Contient les configurations pour lier les senseurs 3D et les différents paramètres de moveit.

## Launch files  
* `moveit_master.launch`  
*Description*  
    * Sert à lancer les différents packages de moveit.  
* `setup_assistant.launch`  
*Description*  
    * Lance l'outil de configuration de moveit. Le fichier (flying_box.urdf.xacro) sur lequel les configurations actuelles sont basées se trouve dans elikos_nav/cfg.  
* Plusieurs autres sans importance.

## Configurations
* `sensors_intel_realsense.yaml`  
*Description*  
    * Configure le(s) PointCloud(s) reçu(s) du/des caméra(s) Intel Realsense r200.  
    * topics des PointCloud(s) :  
        * `/r200/depth/points`  
* `controllers.yaml`  
*Description*  
    * Définit le type de trajectoire généré pour chaque jointure du modèle (.urdf).
* Plusieurs autres sans importance.

## À savoir!
* Le principal launch file (`moveit_master.launch`) est lancé dans `nav.launch` du package **elikos_nav**. 