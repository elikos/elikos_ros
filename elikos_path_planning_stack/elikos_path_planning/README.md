# ELIKOS_PATH_PLANNING 
## Buts  
* Package principal du path planning. 
* Recevoir les commandes de l'AI et les envoyer dans le planning pipeline de moveit.
* Obtenir la trajectoire à exécuter de moveit et envoyer une commande de position à mavros.

## Launch files  
* `path_planning.launch`  
*Description*  
    * Sert à lancer tous les éléments du path planning.  

## Nodes  
* **elikos_moveit_move_group** : Node central pour utiliser moveit. 

## Input topics  
* `elikos_decisionmaking_cmd`  
    * type : `geometry_msgs::PoseStamped`  
    * desc : commande de position envoyée par l'AI. 

## Output topic  
* `elikos_trajectory`  
    * type : `elikos_msgs::TrajectoryCmd`    
    * desc : Commande et trajectoire à destination du contrôle.  