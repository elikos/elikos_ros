# ELIKOS_DECISIONMAKING  
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
    *  Lance le node elikos_decisionmaking 
    *  TODO: Spécification du fichier de configuration 
* `simulation.launch`   
*Description*   
    * Lance le node de simulation qui du package elikos_sim
    * Lance l'émulateur qui traduit les infos (setpoints, positions,etc.) vers les topics et frames tf attendus par le 
      node ai et simule également la restriction du champs de vision des caméras.
    
## Configuration   
* dimension_c: (default = 3)  
    Dimension d'un côté de l'arène.
* research_altitude: (default = 2)  
    Altitude pour la recherche de cibles.
* edge_tolerance: (default = 0.5)  
    Distance la minimale du drone à un ligne de bordure.
* research_enabled: (default = true)  
    Permet au drone de chercher des cibles.
* aggressive_enabled: (default = false)  
    Permet au drone de faire passer les cibles de l'autre côté de la ligne verte.
* preventive_enabled: (default = false)  
    Permet au drone d'éviter les cibles de passer de l'autre côté d'une ligne rouge.
* simulation: (default = false)  
    Indique si on veut exécuter la simulation de l'AI.

## Nodes  
* **elikos_decisionmaking** : Effectue la prise de décision haut niveau. Génère des destinations et devrait 
                 éventuellement générer les commandes.  
* **elikos_emulator** : Transforme les coordonnées du repère de l'image vers le repère de l'arène.  

## Input topics  
TODO: 

## Output topics  
TODO  
