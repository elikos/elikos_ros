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

## Nodes  
* **elikos_origin_init** : Permet d'initialiser le référentiel tf `elikos_arena_origin` à l'aide d'un service.  
ROS service: `elikos_origin_init`. Aucun argument.