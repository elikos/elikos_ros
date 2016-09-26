# ELIKOS_AI  
## Buts  
* Prise de décision haut niveau.  
* Amener le plus de robots cibles possible à la ligne verte dans la limite de temps donnée.  

## Stratégies  
* Machine à états sous forme de comportements 
* Génération et exécution d'une série de commandes 
* Calcul de priorité des robots cibles

## Launch files  
* `ai.launch`  
*Description*  
    *  Lance le node elikos_ai 
    *  TODO: Spécification du fichier de configuration 
* `simulation.launch`   
*Description*   
    * Lance le node de simulation qui du package elikos_sim
    * Lance l'émulateur qui traduit les infos (setpoints, positions,etc.) vers les topics et frames tf attendus par le 
      node ai et simule également la restriction du champs de vision des caméras.
    
## Configuration   
TODO

## Nodes  
* **elikos_ai** : Effectue la prise de décision haut niveau. Génère des destinations et devrait 
                 éventuellement générer les commandes.  
* **elikos_emulator** : Transforme les coordonnées du repère de l'image vers le repère de l'arène.  

## Input topics  
TODO: 

## Output topics  
TODO  
