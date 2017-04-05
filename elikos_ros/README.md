# ELIKOS_ROS  
## One package to rule them all!

## Launch files  
* `elikos_software_stack.launch`  
*Description*  
    * Lance les différents packages du repo.  
* `elikos_drivers.launch`  
*Description*  
    * Lance les caméras, le px4 et autres drivers.  
* `px4.launch`  
*Description*  
    * Lance le px4 avec les configurations spécifées dans `px4_config.yaml`.  
* `cameraX.launch`  
*Description*  
    * Lance la caméra X.  
* `elikos_origin_init.launch`  
*Description*  
    * Lance le node **elikos_origin_init**.  
* `calibration.launch`  
*Description*  
    * Lance une instance du node de détection en mode calibration ainsi qu'une caméra.  
* `elikos2016.launch`  
*Description*  
    * Launch file utilisé à la compétition 2016.  
* `camera.launch`  
    * Lance une camera avec le package `pointgrey_camera_driver`  
    * Paramètres :  
        * `camera_serial` le numéro de série de la caméra trouvable avec 
		```
        rosrun pointgrey_camera_driver list_cameras
        ```
        * `calibrated` 1 si la caméra doit être calibrée, sinon 0
        * `camera_number` le numéro de la caméra (pour la calibration)

## Nodes  
* **elikos_origin_init** : Permet d'initialiser le référentiel tf `elikos_arena_origin` à l'aide d'un service.  
ROS service: `elikos_origin_init`. Aucun argument.
