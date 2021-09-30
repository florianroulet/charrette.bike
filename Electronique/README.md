# Electronique

Sera évoquée ci-dessous l'électronique de l'assistance électrique utilisant un capteur de force. Il s'agit du Charduino, contraction de Charrette et Arduino.


Pour rappel voici un schéma de principe:

![Schéma de principe](/img/principe_assistance_capteur.png)


## Généralités

Le charduino est le circuit imprimé qui gère l'électronique de la charrette.
Les composants principaux du système sont:
 * Un arduino nano
    * Il traite toutes les données venant des différents capteurs et envoie les ordres au contrôleur
 * Un contrôleur de moteur électrique
    * Il active le moteur ou le frein moteur en fonction du signal de la gachette et du signal de frein
    * Ces deux signaux sont générés par l'arduino, en fonction des capteurs
 * Une batterie 36 ou 48V
    * Plomb ou lithium
    * 48V pour avoir plus de couple
 * Un moteur électrique de vélo
    * 1200W pour assurer de déplacer 300kg
 * Un capteur de force
    * Des jauges de déformation qui quantifient la compression ou la traction de la barre d'attelage.
    * Cette information est envoyée à l'arduino
 * Les capteur effet hall du moteur
    * Capteurs intégrés dans le moteur.
    * Ils renseignent sur la vitesse et le sens de rotation de la roue.
 * Un boîtier de contrôle
    * Pour renseigner l'utilisateur sur l'état du système
    * Pour sélectionner différents mode de fonctionnement (activation, frein, mode piéton)


## Fonctionnement d'un vélo électrique
Un vélo électrique fonctionne grâce à
 * Un vélo
 * Une batterie
 * Un moteur
 * Un contrôleur
 * Des accessoires de commande qui peuvent être:
    * Une gachette d'accélération
    * Un frein
    * Un capteur de pédalage

Le contrôleur agit comme chef d'orchestre, il est relié à la batterie, au moteur et aux accessoires de commande.
En fonction de ces derniers et des signaux envoyés, le contrôleur transforme le courant de la batterie en signaux de puissance triphasé, le moteur tourne alors.

### La gachette

La gachette d'accélération est un potentiomètre, une résistance variable.
Ainsi elle a 3 fils:
 * VCC ou 5V
 * Un signal échelonné entre 0 et 5V
 * GND

Si la gachette n'est pas activé, on envoie 0V au contrôleur, la roue ne tourne pas.
Si la gachette est actionnée au maximum, on envoie 5V au contrôleur, la roue tourne avec toute la puissance possible
Et entre les deux, le contrôleur envoie plus ou moins de puissance dans le moteur.

### Le frein
Le système de frein est basé sur un interrupteur, il y a donc 2 fils:
 * Signal
 * GND

La poignée de frein est équipée d'un interrupteur. Dès que le levier de frein est actionné, le circuit est fermé, le signal est mis à la masse, le contrôleur arrête d'accélérer et active le frein moteur s'il en est capable. Auquel cas la batterie se recharge.

METTRE SCHEMA DE PRINCIPE D'UN CONTROLEUR AVEC LES CONNECTEURS

Pour davantage d'information sur le fonctionnement du vélo électrique, je vous invite à lire [cette page](https://ebikes.ca/getting-started/ebikes-parts-explained.html) en anglais. Le site [ebikes.ca](https://ebikes.ca) est une ressource fiable à recommander.

## Fonctionnement de l'assistance électrique de la charrette

### Considérations légales

Un vélo ne peut être assisté avec plus de 250W sinon il s'agit d'un vélomoteur qui doit être immatriculé et assuré.
Une remorque à vélo peut être motorisée avec davantage de puissance à la condition qu'elle reste un accessoire et qu'elle ne soit pas motrice, c'est à dire qu'elle ne pousse pas le vélo, elle le suit et accompagne le mouvement.

### Solution technique retenue

Pour rentrer dans le cadre légal et rendre la charrette autonome et indépendante du vélo tracteur, la charrette est équipée d'un ***capteur de force*** qui mesure la déformation longitudinale de la barre d'attelage, c'est à dire la compression ou la traction dans le sens de la marche.
Le système repose sur l'idée simple suivante, mettre en équilibre la charrette par rapport au mouvement du vélo en s'appuyant sur le capteur de force et la vitesse de rotation de la roue

### Boîtier de contrôle

Un boîtier de contrôle est fixé sur la barre d'attelage.
Celui-ci permet:
 * d'activer le système
 * de sélectionner un mode d'assistance lent ou normal
 * de forcer l'utilisation du frein moteur
 * de renseigner l'utilisateur sur l'état du système


## Liste composants
### Charduino
| Composant | Description | Quantité | Autre |
| ----------- | ----------- | ----------- | ----------- |
| Charduino | Circuit imprimé principal de la charrette | 1 |
| Arduino Nano | Microcontroleur | 1
| HX711 | Amplificateur pour capteur de force | 1 | Title |
| Relais Omron G5V-2-H1| Activation du contrôleur de moteur | 1 | |
| Transistor NPN générique | activation du frein moteur | 1 | |
| Résistance 5mOhm | Mesure de courant | 1 | |
| Résistance 1kOhm | Pont diviseur de tension | 1 | A vérifier - R5 |
| Résistance 470 kOhm| Amplificateur opérationnel | 2 | R7 - R10|
| Résistance 10 kOhm | Amplificateur opérationnel et pont diviseur de tension | 3 | R6 - R9 - R4 |
| Résistance 2,2kOhm | Tirage par le bas et filtre RC | 3 | R1 - R2 - R3 |
| LM7525HVT| Transformateur 5V | 1 | |
| Condensateur 100µF 63V | Transformateur 5V | 1 | |
| Condensateur 330µF 25V | Transformateur 5V | 1 | |
| Diode 1N5819 | Transformateur 5V | 1 | |
| Bobine 330µH 2A | Transformateur 5V | 1 | |
| MPC602 | Amplificateur opérationnel | 1 | |
| Condensateur 22µF 25V | Filtre RC| 1 | A vérifier |

### Boîtier de contrôle

| Composant | Description | Quantité | Autre |
| ----------- | ----------- | ----------- | ----------- |
| Carte | Circuit imprimé du boîtier de contrôle | 1 | |
| Diode | Affichage utilisateur | 1 | |
| Interrupteur | Interface homme/machine | 3 | |
| Boîte aluminium | Protection | 1 | |
| Presse-étoupe| étanchéïté du câble| 1 | |


### Connectiques

Pour les connectiques internes  de signaux, la gamme JST-SM apporte satisfaction. Ces connecteurs sont déjà utilisés dans le monde des vélos électriques.
Pour les connectiques internes et externes de puissance (phase moteur, batterie), la gamme Anderson est satisfaisante.
Cf [ce document](https://www.ebikes.ca/documents/GrinConnectorGuide.pdf) en anglais de ebikes.ca pour plus d'information
Cette [page](https://ebikes.ca/learn/connectors.html) venant toujours du Canada est intéressante aussi


Pour les connectiques panneaux de signaux, nous utilisons actuellement les connecteurs [Neutrik NC-MX6](https://www.thomann.de/fr/neutrik_nc6mx.htm) utilisés dans le monde du spectacle.

Pour les connectiques panneaux de puissance, nous utilisons les connecteurs [Varytech PM](https://www.thomann.de/fr/varytec_real_pm.htm) aussi utilisés dans l'univers de la régie technique.

En fonction du montage, un nombre plus ou moins grand de ces connecteurs est utilisé.
