# Informatique Charduino

Cette partie documente le code embarqué dans l'arduino qui pilote l'assistance électrique.

## Généralités

Le code est organisé selon une machine à état. Le passage d'un état à l'autre dépend:
 * l'interrupteur de mise en marche
 * l'interrupteur de freinage
 * la vitesse et le sens de rotation de la roue
 * l'effort longitudinal appliqué au timon
 * le courant qui sort de la batterie

 L'idée principale pour régir l'assistance électrique est d'utiliser un PID pour asservir le signal gachette envoyé au contrôleur à l'effort longitudinal lu dans le timon avec les jauges de déformation.
 La lecture de la vitesse et du sens de rotation de la roue sont nécessaires pour les changements d'état.
 Le boîtier de contrôle influe directement via des interrupteurs sur les changements d'état et les paramètres du PID pour le rendre plus ou moins doux.

## PID
## Wattmètre
### Tension
Un pont diviseur de tension est utilisé pour mesurer la tension aux bornes de la batterie. Cette information est utile pour limiter l'usage de l'assistance à une plage de tension et ainsi préserver la batterie de décharge profonde. Cette sécurité est censée être intégrée dans les contrôleurs de moteur, il s'agit là d'une redondance de sécurité.
### Courant
Une résistance de shunt est utilisée avec un amplificateur opérationnel pour mesurer le courant sortant de la batterie.
Cette information est utile dans le cas où le contrôleur se met en défaut malgré l'envoi d'une commande par le signal de la gachette.
Ce cas peut se produire en montée et avec un chargement lourd. Le contrôleur n'arrive pas à faire tourner la roue, il envoie plus de courant mais rien n'y fait. Le contrôleur se met en défaut.
Pour détecter ce cas, qui se produit davantage en utilisant un moteur sans capteur à effet hall, il est nécessaire de lire le courant de sortie de la batterie.
### Puissance
On en déduit plus ou moins finement la puissance consommée par le système.
## Lecture vitesse et rotation de la roue
On utilise les capteurs à effet hall embarqués dans le moteur pour savoir comment tourne la roue.

Initialisation / setup:
  * Ouverture port série
  * Cartouche de bienvenue
  * Iniitialisation des entrées/sorties
  * Initialisation des interruptions
  * Initialisation du PID
  * Initialision du wattmètre
  * Initialisation des différents chronomètres

Boucle principale:
  * Mise à jour de la vitesse
  * Lecture de la valeur du capteur de force
  * Lecture de la tension et du courant de sortie de batterie
  * Gestion des "software debounce" pour les interrupteurs du boîtier de contrôle
  * Gestion de la machine à état
  * Impression des messages de debug
  * Vérification de l'état du contrôleur
  * Envoi de l'ordre de la gachette
7 états

Etat 0: Initialisation
Etat 1: Attente avant départ
Etat 2: Roulage
Etat 3: En suspens
Etat 4: Déccélération
Etat 5: Freinage
Etat 6: ?
Etat 7: Réinitialisation du capteur

0 -> 1
  Conditions:
    Etat == Initialisation
    Initialisation du capteur réussie
    Vitesse moyenne < 1km/h
    Contrôleur allumé
  Actions:

1 -> 2
  Conditions:
    Etat



* -> 7
  Conditions:
    Etat == *
    3 commutations succcessives de l'interrupteur "Frein"
    Contrôleur éteint
