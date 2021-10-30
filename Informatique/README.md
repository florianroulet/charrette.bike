Charduino

Soit la méthode avec capteur

Soit la méthode avec contacteur

Allons pour la méthode avec contacteur
Mettre un filtre passe-bas c'est bien à l'ouverture du contacteur situé avant la résistance mais pas pour la fermeture.
Faire un filtre-bande c'est mieux.
On va déjà faire un filtre passe-bas

https://www.translatorscafe.com/unit-converter/fr-FR/calculator/rc-circuit/


Pour la gachette:
 R=100 Ohm
 C=330µF
 T=RC=33ms


Pour le frein
On positionne un filtre RC passe-bas devant le transistor qui va servir d'interrupteur pour relier les deux fils du frein (SIG et GND)
Le transistor est passant pour une tension Vbe>0,7V
Il faut calculer le couple RC pour avoir un temps de 1 seconde pour que le condensateur atteigne 0,7V.
Dans l'autre sens quel temps aura-t-on à la décharge pour pour descendre en dessous de 0,7V et couper le frein?
