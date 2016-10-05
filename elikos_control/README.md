# ELIKOS_CONTROL  
## Buts  
* Décollage et atterrissage automatiques.  
* Exécution des trajectoires.  

## Stratégies  
* Contrôle par position/vitesse/accélération.  

## Launch files  
* `elikos_control.launch`   
*Description*   
    * Lance une instance du package de contrôle.  
  
## Nodes  
* **elikos_control** : Décollage et atterrissage automatiques. Exécution des trajectoires.  

## Input topics  
* `elikos_trajectory`  
    * type : `elikos_ros::TrajectoryCmd`    
    * desc : Commande et trajectoire.  