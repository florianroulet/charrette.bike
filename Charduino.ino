#include <EEPROM.h>
#include <Chrono.h>
#include "MoteurEBike.h"
#include <PID_v1.h>
#include "RemorqueLed.h"
#include <SoftFilters.h>
#include "PinChangeInterrupt.h"
#include "wattmeter.h"
#include "StrengthSensor.h"


/*
 * Paramètres sur lesquels influer:
 *
 *
 *
 *
 *
 */


/////////////////////////////////////////////////
/////////////// Debug ///////////////////////////
/////////////////////////////////////////////////

bool debugPython = 0;     // pour envoyer les données au format attendu par le script python
bool debug = 1;           // pour envoyer les données de debug au format texte dans le moniteur série
bool debugCsv=  0;        // pour envoyer les données de debug au format csv dans le moniteur série
bool debugMotor = 0;      // si aucun moteur n'est branché, pour le simuler
bool debugFrein = 0;      // si aucun interrupteur n'est branché, sur le frein à inertie. Inutilisé actuellement
bool debugCapteur = 0;    // si aucun capteur de force n'est branché, pour le simuler
bool debugOther = 0;      // utilisé à des fins de tests.
bool old = 0;             // pour assurer la compatibilité avec une ancienne version de carte électronique. A garder à 0.
int csvIter = 0;          // compteur d'itération pour l'affichage csv.

/////////////////////////////////////////////////
/////////////// capteur force ///////////////////
/////////////////////////////////////////////////

const int HX711_dout = 7 ; //mcu > HX711 dout pin
const int HX711_sck = 8; //mcu > HX711 sck pin
double capteur_offset = 841.86;

StrengthSensor capteur(HX711_dout, HX711_sck, capteur_offset);


// pour calculer la dérivée du capteur de force. Inutilisé
DifferentialFilter<double, unsigned long> diffCapteur;
Reading<double, unsigned long> rCapteur;
Reading<Differential<double>, unsigned long> dCapteur;


double valeurCapteur;
bool newDataReady = 0;
bool capteurInitialise = 0;
Chrono resetOffsetChrono; // Chrono pour ?
int resetOffsetIter;			// compteur pour compter le nombre d'aller retour sur l'interrupteur brake
MovingAverageFilter<double, double> movingOffset(16); // moyenne lissée sur 16 valeurs
double newOffset, rawValue;


/////////////////////////////////////////////////
/////////////////// EEPROM  /////////////////////
/////////////////////////////////////////////////

// on utilise la mémoire eeprom pour stocker la valeur à considérer comme 0 pour le capteur de force.

int eeprom = 0;

/////////////////////////////////////////////////
/////////////// Frein à inertie /////////////////
/////////////////////////////////////////////////

/*
 * Inutilisée
 *
 * Pour ajouter de la sécurité, l'idée est de mettre un contacteur au niveau du frein à inertie.
 * Quand ça s'ouvre, la remorque commence à aller plus vite que le vélo.
 * L'information est plus basique que celle du capteur de force mais permet d'ajouter une redondance d'informations.
 * Il n'a pas été trouvé encore de position correcte ou de contacteur fiable et robuste.
 * Idée en repos, mais importante à mettre en oeuvre par la suite.
 */

const int frein = debugFrein ? 4 : 5;
Chrono chronoFrein;
bool freinFlag = 0;
int t1 = 300;
int t2 = 1000;
int t3 = 1500;




/////////////////////////////////////////////////
/////////////// Moteur //////////////////////////
/////////////////////////////////////////////////

/*
 * Objet permettant de piloter le moteur au travers du signal de gachette et de frein
 */

MoteurEBike moteur = MoteurEBike();

/////////////////////////////////////////////////
///////////////// PID ///////////////////////////
/////////////////////////////////////////////////

/*
 * Pour lier l'information du capteur de force au signal PWM envoyé dans la gachette, on utilise un PID.
 * https://playground.arduino.cc/Code/PIDLibrary/
 */

// Paramètres à changer:

	double K1[3] = {1, 4, 0.1}; // boost, mode 0 pour les led
	double K2[3] = {1, 2, 0.1}; //marche, mode 1 pour les led
	float betaTab[2]={-6,-3};
	float gammaTab[2]={-9,-6};
	double consigneCapteurTab[2]={-2.0,0.0};

	double sortieMoteur;    //output
	double consigneCapteur = consigneCapteurTab[1]; //setpoint, valeur visée par le PID comme valeur de capteur.
	float pwmMin = 110, pwmMax = 255; // les valeurs minimales et maximales pour le PWM de la gachette.
		                          //110 a été trouvée expérimentalement avec une batterie 48V et un contrôleur Ozo. En deça la roue ne tourne pas.
		                          // ces valeurs sont à tester et corriger en cas de changement de batterie ou contrôleur.

	PID myPID(&valeurCapteur, &sortieMoteur, &consigneCapteur, K1[0], K1[1], K1[2], P_ON_E, REVERSE);
    // Entrée: ValeurCapteur
    // Valeur asservie: SortieMoteur, qui est un PWM allant de pwmMin à pwmMax
    // ConsigneCapteur: Valeur visée pour ValeurCapteur
    // Kp,Ki,Kd, les paramètres dont dépendent l'asservissement du pwm
    // P_ON_E, Proportionnal on Error
    // REVERSE, augmenter la valeur asservie, diminuera l'entrée.

/////////////////////////////////////////////////
//////////////// Etats //////////////////////////
/////////////////////////////////////////////////

enum etats_enum {INITIALISATION,  //0
                 ATTENTE,         //1
                 ROULE,           //2
                 STATU_QUO,       //3
                 DECCELERATION,   //4
                 FREINAGE,        //5
                 MARCHE,          //6
                 RESET_CAPTEUR    //7
                };

int etat = INITIALISATION;

// Ancre paramètre 2

float alpha = 1;  //seuil au dessus duquel le PID se calcule et se lance
float beta = betaTab[1] ; //seuil en deça duquel on passe sur déccélération (pwm=0, pid manual)
float gamma = gammaTab[1]; // seuil en deça duquel on passe sur du freinage


/////////////////////////////////////////////////
//////////// Vitesse moyenne ////////////////////
/////////////////////////////////////////////////

/*
 * Stockage de la vitesse du moteur.
 * Vitesse lue grâce aux capteurs à effet hall du moteur
 * En cas de défaillance de cette information, le système n'est plus utilisable
 */

MovingAverageFilter<double, double> vitesseFiltree(4);
double vitesseMoyenne;
double vitesseInstantanee;



/////////////////////////////////////////////////
///////////// Pin interrupteur debug ////////////
/////////////////////////////////////////////////

int plus = 2;
int moins = 3;
int halt = 4;
bool haltFlag = 0;


/////////////////////////////////////////////////
///////////// Pin controleur ////////////////////
/////////////////////////////////////////////////

int ctrlAlive = 12;     // sur cette pin arrive le 5V de la gachette. Cette information nous renseigne sur l'état du contrôleur.
int ctrlSwitch = 13;    // pin utilisée pour allumer le relais qui activait le contrôleur. Inutilisée depuis que le relais est activé en direct par un interrupteur. A nettoyer
bool isCtrlAlive = debugMotor ? 1 : 0;

/////////////////////////////////////////////////
///////////////// LED  //////////////////////////
/////////////////////////////////////////////////

float maxTraction = 20.0;
float seuilTractionNulle = 0.3;

RemorqueLed led = RemorqueLed(maxTraction, seuilTractionNulle, pwmMin, pwmMax);


/////////////////////////////////////////////////
///////////////// Wattmetre /////////////////////
/////////////////////////////////////////////////

Wattmeter wattmetre;

int amPin = old ? A4 : A6; // select the input pin for the potentiometer  vert
int vPin = old ? A5 : A7; // select the input pin for the potentiometer   jaune
float amCalib = 28.84;
float vCalib = 22.41;

bool isFlowing;
Chrono flowingChrono; // chrono pour s'avoir depuis quand on envoie un pwm
Chrono stoppedChrono; // chrono pour s'avoir depuis quand on a détecté un arrêt.
int flowingState;



int powerPin = old ? 0 : A3; // pin pour allumer le controleur            Jaune
int motorBrakePin = old ? 0 : A4; // pin pour activer le mode freinage    Vert
int walkPin = old ? 0 : A5; // pin pour activer le mode piéton;           Rouge
Chrono powerChrono, motorBrakeChrono, walkChrono;
bool powerNewState, motorBrakeNewState, walkNewState = 0;
int debounceTime = 100;



bool powerCtrl = old ? 1 : 0;
bool walkMode = 1;
bool motorBrakeMode = 0;



void setup()
{

  Serial.begin(9600);

  Serial.println("###################");
  Serial.println("## version 0.9.5: ");
  Serial.println("## date: 20/07/2021: ");
  Serial.println("## MAAD93 ");
  Serial.println("###################");


  // interrupteur sur la carte
  pinMode(plus, INPUT_PULLUP);
  pinMode(moins, INPUT_PULLUP);
  pinMode(halt, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(plus), pluss, CHANGE);
  attachPCINT(digitalPinToPCINT(moins), moinss, CHANGE);
  attachPCINT(digitalPinToPCINT(halt), haltt, CHANGE);

  // freinage inertiel
  pinMode(frein, INPUT_PULLUP);
  chronoFrein.stop();
  attachPCINT(digitalPinToPCINT(frein), freinage, CHANGE);


  // lecture vitesse
  attachPCINT(digitalPinToPCINT(moteur.getUPin()), interruptU, CHANGE);
  attachPCINT(digitalPinToPCINT(moteur.getVPin()), interruptV, CHANGE);

  //PID
  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(pwmMin, pwmMax);
  myPID.SetSampleTime(200);

  // Controleur
  pinMode(ctrlAlive, INPUT);
  pinMode(ctrlSwitch, OUTPUT);
  digitalWrite(ctrlSwitch, false);
  attachPCINT(digitalPinToPCINT(ctrlAlive), interruptCtrlAlive, CHANGE);

  // Wattmetre
  wattmetre = Wattmeter(amPin, vPin, amCalib, vCalib, 36);

  flowingChrono.restart();
  flowingChrono.stop();
  stoppedChrono.restart();
  stoppedChrono.stop();

  led.ledWelcome();
  led.setMode(walkMode);

  //capteur
  EEPROM.get(eeprom, capteur_offset);
  Serial.print("## offset: "); Serial.println(capteur_offset);
  capteur.setOffset(capteur_offset);
  capteur.setSamplesInUse(8); //16 le défaut

  pinMode(powerPin, INPUT_PULLUP);
  pinMode(motorBrakePin, INPUT_PULLUP);
  pinMode(walkPin, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(powerPin), powerPinInterrupt, CHANGE);
  attachPCINT(digitalPinToPCINT(motorBrakePin), motorBrakePinInterrupt, CHANGE);
  attachPCINT(digitalPinToPCINT(walkPin), walkPinInterrupt, CHANGE);
  powerChrono.restart();
  powerChrono.stop();
  motorBrakeChrono.restart();
  motorBrakeChrono.stop();
  rCapteur.timestamp = 0;
}

void loop()
{
  // checkCtrl();
  miseAJourVitesse();
  if (capteurInitialise) {
    capteur.update(&newDataReady, &valeurCapteur);
    rCapteur.value = valeurCapteur * 1000;
    rCapteur.timestamp += millis() / 1000;
    diffCapteur.push(&rCapteur, &dCapteur);
  }

  if (valeurCapteur > 30) {
    Serial.println("ERREUR ELM?");
    // valeurCapteur=0;
  }

  wattmetre.update();

  if (powerChrono.elapsed() > debounceTime && digitalRead(powerPin) == powerNewState) {
    powerCtrl = powerNewState;
    powerChrono.restart();
    powerChrono.stop();
    switchCtrl(powerCtrl);
  }

  if (motorBrakeChrono.elapsed() > debounceTime && digitalRead(motorBrakePin) == motorBrakeNewState) {
    motorBrakeMode = motorBrakeNewState;
    motorBrakeChrono.restart();
    motorBrakeChrono.stop();
  }






  ////////////////////////////////////////////////////:
  ///////////////////  0  ////////////////////////////
  ////////////////////////////////////////////////////:

  if (etat == INITIALISATION) {
    led.ledPrint(valeurCapteur, sortieMoteur);

    moteur.setMoteurState(STOPPED);
    capteur.setThresholdSensor(0.5);
    switchCtrl(powerCtrl);


		// Si ça fait plus de 1000ms que le chrono est lancé, on ne veut pas réinitialiser le capteur, remise à 0.
    if (resetOffsetChrono.elapsed() > 1000) {
      resetOffsetIter = 0;
      resetOffsetChrono.stop();
    }

    if (transition5()) {
      etat = FREINAGE;
    }
    else if (transition01()) {
      etat = ATTENTE;
    }
    else if (transition07()) {
      etat = RESET_CAPTEUR;
    }
  }
  ////////////////////////////////////////////////////:
  ///////////////////  1  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if ( etat == ATTENTE) {

    myPID.SetMode(MANUAL);
    led.ledState(etat);
    moteur.setMoteurState(STOPPED);
    capteur.setThresholdSensor(0.5);


		// En mode 1, on arrête le chrono et remet à 0 les itérations pour interdire le reset capteur.
    resetOffsetIter = 0;
    resetOffsetChrono.stop();

    if (flowingChrono.isRunning()) {
      flowingChrono.restart();
      flowingChrono.stop();
    }

    if (transition5()) {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition12())
      etat = ROULE;
    else if (transition15())
      etat = FREINAGE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  2  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == ROULE) {
    moteur.setMoteurState(SPINNING);
    myPID.SetMode(AUTOMATIC);
    miseAJourPID();
    if (!flowingChrono.isRunning()) {
      flowingChrono.restart(); //
    }
    /*
      if(wattmetre.getState()==3)
      led.ledPrint(valeurCapteur,sortieMoteur);
      else{
      led.ledFail(2);
      }
    */
    led.ledPrint(valeurCapteur, sortieMoteur);
    capteur.setThresholdSensor(0.0);
    //   flowingOrNot();


    if (transition5()) {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else if (transition23())
      etat = STATU_QUO;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  3  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if ( etat == STATU_QUO) {
    moteur.setMoteurState(SPINNING);

    led.ledState(etat);
    /*
        if(wattmetre.getState()==3)
          led.ledPrint(valeurCapteur,sortieMoteur);
        else{
          led.ledFail(3);
        }
    */
    led.ledPrint(valeurCapteur, sortieMoteur);
    // myPID.SetMode(MANUAL);
    // flowingOrNot();


    myPID.SetMode(AUTOMATIC);
    miseAJourPID();


    if (transition5()) {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else  if (transition32())
      etat = ROULE;
    else if (transition34())
      etat = DECCELERATION;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  4  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if ( etat == DECCELERATION) {

    led.ledState(etat);
    moteur.setMoteurState(STOPPED);
    //moteur.setMoteurState(SPINNING);
    myPID.SetMode(MANUAL);
    sortieMoteur = pwmMin;
    // myPID.SetMode(AUTOMATIC);
    // miseAJourPID();

    //   if(flowingChrono.isRunning()){
    //      flowingChrono.restart();
    //      flowingChrono.stop();
    //    }

    if (transition5()) {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else  if (transition42())
      //  etat = ROULE;
      etat = ATTENTE;
    else if (transition45())
      etat = FREINAGE;
  }


  ////////////////////////////////////////////////////:
  ///////////////////  5  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == FREINAGE) {

    led.ledState(etat);
    myPID.SetMode(MANUAL);
    moteur.setMoteurState(BRAKING);
    sortieMoteur = pwmMin;
    capteur.setThresholdSensor(0.5);

		// En mode 5, on arrête le chrono et remet à 0 les itérations pour interdire le reset capteur.
    resetOffsetIter = 0;
    resetOffsetChrono.stop();

    if (flowingChrono.isRunning()) {
      flowingChrono.restart();
      flowingChrono.stop();
    }

    if (transition5()) {
      etat = FREINAGE;
    }
    else if (transition0())
      etat = INITIALISATION;
    else   if (transition52())
      etat = ATTENTE; //ROULE;
    else if (transition51())
      etat = ATTENTE;
  }

  ////////////////////////////////////////////////////:
  ///////////////////  7  ////////////////////////////
  ////////////////////////////////////////////////////:

  else if (etat == RESET_CAPTEUR) {
    /*
       On attend 500ms puis la valeur actuelle du capteur est stockée et lissée sur 32 valeurs
    */


		/*
			Je comprends pas le rapport au nombre 10.
		*/
    if (resetOffsetIter != 10)
      resetOffsetChrono.restart();
    resetOffsetIter = 10;
    if (resetOffsetChrono.elapsed() > 500) {
      if (millis() % 500 > 250)
        led.ledFail(etat);
      else
        led.ledFail(etat - 1);
      rawValue = capteur.getRaw();
      movingOffset.push(&rawValue, &newOffset);
    }
    if (transition70()) {
      //  Serial.print(" ## capteur valeur: ");Serial.println(newOffset);
      EEPROM.put(eeprom, newOffset);
      capteur.setOffset(newOffset);
      resetOffsetIter = 0;
      resetOffsetChrono.restart();
      resetOffsetChrono.stop();
      etat = INITIALISATION;
    }
  }


  else {
    //  Serial.println("Sortie de cas");
    led.ledFail(OTHER);
  }

  /*
    if(etat==ROULE){
     if(millis()%2000>1000){
       sortieMoteur=240;
       moteur.setMoteurState(SPINNING);
     }
     else{
       sortieMoteur=0;
       moteur.setMoteurState(BRAKING);
       }
    }

  */

 //flowingOrNot();


  if (debugPython) {

    envoi(vitesseMoyenne);
    envoi(sortieMoteur);
    envoi(valeurCapteur);
    envoiInt(myPID.GetKi());
    envoiFin();
  }
  else if (debugCsv) {
    printCsv();
  }
  else if (debug) {
    debugMessage();
  }
  else {

  }
  moteur.mettreLesGaz(sortieMoteur);


}


bool transition01() {
  //Serial.print("transition 01"); Serial.print(" - "); Serial.print(etat == INITIALISATION); Serial.print(" - "); Serial.print(initialisationCapteur()); Serial.print(" - "); Serial.print(vitesseMoyenne < 1.0); Serial.print(" - "); Serial.print(isCtrlAlive);
  //Serial.println();
  return (etat == INITIALISATION && initialisationCapteur() && vitesseMoyenne < 1.0 && isCtrlAlive);
}
bool transition07() {
  return (etat == INITIALISATION && resetOffsetIter > 3 && !isCtrlAlive);
}
bool transition12() {
  return (etat == ATTENTE && valeurCapteur > alpha && vitesseMoyenne > 1.0 && isCtrlAlive);
}
bool transition15() {
  return (etat == ATTENTE && valeurCapteur < gamma || chronoFrein.elapsed() > t1 && isCtrlAlive);
}
bool transition23() {
  return (etat == ROULE && (valeurCapteur < 0.5 || chronoFrein.elapsed() > t1) && vitesseMoyenne > 0 && isCtrlAlive);
}
bool transition32() {
  return (etat == STATU_QUO && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition34() {
  return (etat == STATU_QUO && (valeurCapteur < beta && valeurCapteur > gamma ) || chronoFrein.elapsed() > t2 && isCtrlAlive);
}
bool transition42() {
  return (etat == DECCELERATION && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition45() {
  return (etat == DECCELERATION && valeurCapteur < gamma || chronoFrein.elapsed() > t3 && isCtrlAlive);
}
bool transition52() {
  return (etat == FREINAGE && (valeurCapteur > 2 * alpha || (valeurCapteur > alpha && vitesseMoyenne < 1.0 )) && !chronoFrein.isRunning() && isCtrlAlive && !motorBrakeMode);
}
bool transition51() {
  return (etat == FREINAGE && !chronoFrein.isRunning() && vitesseMoyenne < 1.0  && valeurCapteur >= 0.0 && valeurCapteur < alpha && isCtrlAlive  && !motorBrakeMode);
}
bool transition70() {
  return (etat == RESET_CAPTEUR && !isCtrlAlive && !motorBrakeMode && resetOffsetChrono.elapsed() > 3000);
}
bool transition0() {
  return (!isCtrlAlive || !initialisationCapteur());
}

bool transition5() {
  return (isCtrlAlive && (motorBrakeMode || valeurCapteur < gamma));
}


/*
   interruptions
*/

void interruptU()
{
  moteur.interruptU();
}

void interruptV()
{
  moteur.interruptV();
}
void interruptCtrlAlive()
{
  if (digitalRead(ctrlAlive))
    isCtrlAlive = 1;
  else
    isCtrlAlive = 0;
}


void freinage() {
  if (debugFrein) {
    if (!digitalRead(frein) && !chronoFrein.isRunning()) {
      chronoFrein.start();
    }
    else if (digitalRead(frein) && chronoFrein.isRunning()) {
      chronoFrein.restart();
      chronoFrein.stop();
    }

    if (!digitalRead(frein)) {
      vitesseMoyenne = vitesseMoyenne ? 0.0 : 10.0;
    }
  }
  else {
    if (digitalRead(frein) && !chronoFrein.isRunning()) {
      chronoFrein.start();
    }
    else if (!digitalRead(frein) && chronoFrein.isRunning()) {
      chronoFrein.restart();
      chronoFrein.stop();
    }
  }
}
void pluss() {
  //if((debugCapteur || debugOther) && !digitalRead(plus))
  //  Kd+=0.1;
}
void moinss() {
  // if((debugCapteur || debugOther) && !digitalRead(moins))
  //   Kd-=0.1;
}
void haltt() {
  // if(!digitalRead(halt)){
  //   vitesseMoyenne=vitesseMoyenne?0.0:10.0;
  // }
}


void powerPinInterrupt() {
  if (digitalRead(powerPin)) {
    powerNewState = true;
    powerChrono.restart();
    //powerCtrl=1;
  }
  else {
    powerNewState = false;
    powerChrono.restart();
  }


  // switchCtrl(powerCtrl);
}
void motorBrakePinInterrupt() {
  /*
      Interruption pour activer le frein moteur

      cette interruption sera utilisée aussi pour réinitialiser le zéro du capteur de force quand le controleur est éteint.
      L'activer 3 fois de suite fera passer dans l'état 6 qui est celui de recalcul du zéro du capteur de force.
  */

  if (digitalRead(motorBrakePin)) {
    //motorBrakeMode=1;
    motorBrakeNewState = true;
    motorBrakeChrono.restart();

    // reset capteur de force
		// Quand on active le frein moteur, si en mode 0
    if (powerCtrl == 0 && etat == INITIALISATION) {
      resetOffsetChrono.restart();
      resetOffsetIter++;
    }
  }
  else {
    motorBrakeNewState = false;
    motorBrakeChrono.restart();
  }
}
void walkPinInterrupt() {
  if (digitalRead(walkPin)){
    walkMode = 1;
    consigneCapteur = consigneCapteurTab[1];
    beta = betaTab[1];
    gamma = gammaTab[1];
    myPID.SetOutputLimits(pwmMin, 220);

  }
  else{
    walkMode = 0;
    beta = betaTab[0];
    gamma = gammaTab[0];
    consigneCapteur = consigneCapteurTab[0];
    myPID.SetOutputLimits(pwmMin, pwmMax);

  }

  setPIDMode(walkMode);
}


/*
   Fonctions annexes
*/

void setPIDMode(bool walkOrNot) {
  if (walkOrNot) {
    myPID.SetTunings(K2[0], K2[1], K2[2]);
    led.setMode(1);
  }
  else {
    myPID.SetTunings(K1[0], K1[1], K1[2]);
    led.setMode(0);
  }
}

int initialisationCapteur() {
  if (!capteurInitialise) {
    if (debugCapteur) {
      capteurInitialise = 1;
      return 1;
    }
    else {
      //capteur
      capteur.begin();
      if (capteur.start()) {
        Serial.println("Initialisation du capteur réussie");
        capteurInitialise = 1;
        return 1;
      }
      else {
        Serial.println("Initialisation du capteur échouée. Vérifier connexion");
        capteurInitialise = 0;
        return 0;
      }
    }
  }
  else return 1;
}

void flowingOrNot() {

  /*
     Pas bon, si ça bloque dès le démarrageet qu'on passe pas en Flowing
  */

  if (wattmetre.getCurrent() > 0.0) {
    if (!flowingChrono.isRunning())
      flowingChrono.restart();
    if (stoppedChrono.isRunning()) {
      stoppedChrono.restart();
      stoppedChrono.stop();
    }
    isFlowing = 1;
    flowingState = 1;
  }
  else {
    if (flowingChrono.elapsed() > 1000) {
      isFlowing = 0;
      flowingChrono.restart();
      flowingChrono.stop();
      flowingState = 2;
    }
    if (isFlowing == 0) {
      if (etat == 2 || etat == 3) {
        if (!stoppedChrono.isRunning()) {
          stoppedChrono.restart();
          moteur.setMoteurState(STOPPED);
          flowingState = 3;
        }
        else if (stoppedChrono.elapsed() < 300) {
          flowingState = 4;
        }
        // la condition suivante est pas idéale pck si on loupe le coche que se passe-t-il?
        else if (stoppedChrono.elapsed() > 300 && stoppedChrono.elapsed() < 1000) {
          moteur.setMoteurState(SPINNING);
          sortieMoteur = 120;
          flowingState = 5;
        }
        else {
          stoppedChrono.restart();
          flowingState = 6;
        }
      }
    }
    else {
      flowingState = 7;
    }
  }

}

void checkCtrl() {
  interruptCtrlAlive();
}

void switchCtrl(bool value) {
  if (!debugMotor) {
    if (value != isCtrlAlive)
      digitalWrite(ctrlSwitch, value);
  }
}

void resetCtrl() {
  if (!isCtrlAlive) {
    switchCtrl(0);
    delay(500);
    switchCtrl(1);
  }
}

void miseAJourVitesse() {
  if (debugMotor) {
    // Serial.println("Oups");
  }
  else {
    moteur.calculerVitesse();
    vitesseInstantanee = moteur.getVitesse();
    vitesseFiltree.push(&vitesseInstantanee, &vitesseMoyenne);
  }
}

void miseAJourPID()
{
  //lectureVitesse = vitesseInstantanee;
  if (!haltFlag)
    myPID.Compute();
}

void debugMessage()
{
  // char data[100];
  // sprintf(data, "Acc:\t %d - Frein:\t %d - Dec:\t %d - Pieton:\t %d - Stop:\t %d - Frein levier: \t %d - t1Overflow: \t %d  \t - vitesse: \t  ",
  //         accelerationFlag, freinageFlag, deccelerationFlag, pietonFlag, stopFlag, freinLaposteFlag, timer1Overflow);
  // Serial.print(data);
  Serial.print(" vitesse :");  Serial.print(vitesseMoyenne);  Serial.print("\t");
  Serial.print("sortie Moteur :");  Serial.print(sortieMoteur);  Serial.print("\t");
//  Serial.print("Moteur state :");  Serial.print(moteur.getMoteurState());  Serial.print("\t");
  Serial.print("Etat :");  Serial.print(etat);  Serial.print("\t");
  //  Serial.print("BRakeMotor pin :");  Serial.print(motorBrakeMode);  Serial.print("\t");  Serial.print(resetOffsetIter);   Serial.print("\t"); //Serial.print(resetOffsetChrono.elapsed());
  //Serial.print("chrono :");  Serial.print(chronoFrein.isRunning());  Serial.print(" : ");  Serial.print(chronoFrein.elapsed());  Serial.print("\t");
//  Serial.print("flowing: "); Serial.print(flowingState); Serial.print("\t");
//  Serial.print("isFlowing: "); Serial.print(isFlowing);  Serial.print(" : ");  Serial.print(flowingChrono.elapsed());  Serial.print("\t");
 // Serial.print("stoppedChrono :"); Serial.print(stoppedChrono.elapsed());  Serial.print("\t ");
  Serial.print("Ampere "); Serial.print(wattmetre.getCurrent());  Serial.print("A\t");
  Serial.print("ampere raw "); Serial.print(wattmetre.getCurrentRaw());  Serial.print("\t");
  Serial.print("Tension: ");Serial.print(wattmetre.getTension());  Serial.print("V\t");
  // Serial.print(wattmetre.getPower());  Serial.print("W\t");
// Serial.print("Wattmetre state "); Serial.print(wattmetre.getState());  Serial.print("\t");
  //  Serial.print("Kd: ");Serial.print(Kd);  Serial.print("\t");
  //  Serial.print("Kp: "); Serial.print(myPID.GetKp());  Serial.print("\t");
    Serial.print("Ki: ");Serial.print(myPID.GetKi());  Serial.print("\t");
  //  Serial.print("Kd: ");Serial.print(myPID.GetKd());  Serial.print("\t");
  Serial.print("Brake: "); Serial.print(digitalRead(motorBrakePin));  Serial.print("\t");
  Serial.print("Walk: "); Serial.print(walkMode);  Serial.print("\t");
  Serial.print("gamma: "); Serial.print(gamma);  Serial.print("\t");
  Serial.print("Mode: ");
  if(walkMode)
    Serial.print("lent");
  else
    Serial.print("rapide");
  Serial.print("\t");


  Serial.print("Capteur :");  Serial.print(valeurCapteur);  Serial.print("\t");//Serial.print(capteur.getRaw());Serial.print("\t");Serial.print(capteur.getReadIndex());Serial.print("\t");
  //Serial.print("rCapteur: "); Serial.print(rCapteur.value);              Serial.print("\t");
  //Serial.print("Timestamp: "); Serial.print(rCapteur.timestamp);          Serial.print("\t");
  // Serial.print("Pos: ");Serial.print(dCapteur.value.position);     Serial.print("\t");
  //Serial.print("Dérivée 1: "); Serial.print(dCapteur.value.speed);        Serial.print("\t");
  //Serial.print("Dérivée 2: "); Serial.print(dCapteur.value.acceleration);


  Serial.println();

}

void printCsv()
{

  if (!csvIter) {
    //  debugMessage();

    Serial.print("Iterateur,");
    Serial.print("vitesse,");
    Serial.print("sortie Moteur,");
    Serial.print("Moteur state ,");
    Serial.print("Etat,");
    Serial.print("flowing,");
    Serial.print("isFlowing,");
    Serial.print("flowingChrono,");
    Serial.print("stoppedChrono,");
    Serial.print("Ampere");
    Serial.print("Tension,");
    // Serial.print("Puissance,");
    Serial.print("Wattmetre state ,");
    Serial.print("Kp,");
    Serial.print("Ki,");
    Serial.print("Kd,");

    Serial.print("Capteur,");
    /*
      Serial.print("rCapteur,");
        Serial.print("Timestamp,Vitesse capteur");
        Serial.print("Vitesse capteur");
        Serial.print("Accel capteur");
    */

  }
  else {

    Serial.print(digitalRead(powerPin));  Serial.print(",");
    Serial.print(isCtrlAlive);  Serial.print(",");
    Serial.print(csvIter);  Serial.print(",");
    Serial.print(vitesseMoyenne);  Serial.print(",");
    Serial.print(sortieMoteur);  Serial.print(",");
    Serial.print(moteur.getMoteurState());  Serial.print(",");
    Serial.print(etat);  Serial.print(",");
    Serial.print(flowingState); Serial.print(",");
    Serial.print(isFlowing);  Serial.print(",");
    Serial.print(flowingChrono.elapsed());  Serial.print(",");
    Serial.print(stoppedChrono.elapsed());  Serial.print(",");
    Serial.print(wattmetre.getCurrent());  Serial.print(",");
    Serial.print(wattmetre.getTension());  Serial.print(",");
    // Serial.print(wattmetre.getPower());  Serial.print(",");
    Serial.print(wattmetre.getState());  Serial.print(",");
    Serial.print(myPID.GetKp());  Serial.print(",");
    Serial.print(myPID.GetKi());  Serial.print(",");
    Serial.print(myPID.GetKd());  Serial.print(",");
    Serial.print(valeurCapteur);  Serial.print(",");//Serial.print(capteur.getRaw());Serial.print(",");Serial.print(capteur.getReadIndex());Serial.print(",");
    /*
      Serial.print(rCapteur.value);              Serial.print(",");
      Serial.print(rCapteur.timestamp);          Serial.print(",");
      Serial.print(dCapteur.value.speed);        Serial.print(",");
      Serial.print(dCapteur.value.acceleration);
    */
  }
  Serial.println();

  csvIter++;


}

void sendPythonIter() {
  envoi(csvIter);
  envoi(sortieMoteur);
}

void debugTransition() {
  Serial.print("0->1: \t"); Serial.print(transition01()); Serial.print(" | ");
  Serial.print("*->0 \t"); Serial.print(transition0()); Serial.print(" | ");
  /*
    Serial.print("1->2: \t");Serial.print(transition12());Serial.print(" | ");
    Serial.print("1->5: \t");Serial.print(transition15());Serial.print(" | ");
    Serial.print("2->3: \t");Serial.print(transition23());Serial.print(" | ");
    Serial.print("3->2: \t");Serial.print(transition32());Serial.print(" | ");
    Serial.print("3->4: \t");Serial.print(transition34());Serial.print(" | ");
    Serial.print("4->2: \t");Serial.print(transition42());Serial.print(" | ");
    Serial.print("4->5: \t");Serial.print(transition45());Serial.print(" | ");
    Serial.print("5->2: \t");Serial.print(transition52());Serial.print(" | ");
    Serial.print("5->1: \t");Serial.print(transition51());
  */
  Serial.println("");

}

void envoi(float vitesse) {
  byte * b = (byte *) &vitesse;
  Serial.write(b, 4);
}
void envoiInt(uint16_t val) {
  byte * b = (byte *) &val;
  Serial.write(b, 2);
}

void envoiChrono(uint64_t val) {
  byte * b = (byte *) &val;
  Serial.write(b, 8);
}

void envoiFin() {
  Serial.write("\n");
}
