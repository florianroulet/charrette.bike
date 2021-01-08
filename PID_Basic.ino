#include <Chrono.h>
#include "MoteurEBike.h"
#include <PID_v1.h>
#include "RemorqueLed.h"
#include <SoftFilters.h>
#include "PinChangeInterrupt.h"
#include "wattmeter.h"
#include "StrengthSensor.h"



/////////////////////////////////////////////////
/////////////// capteur force ///////////////////
/////////////////////////////////////////////////

const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin
const double capteur_offset = 841.86;

StrengthSensor capteur(HX711_dout, HX711_sck, capteur_offset);

double valeurCapteur;
bool newDataReady = 0;
bool capteurInitialise = 0;


/////////////////////////////////////////////////
/////////////// Debug ///////////////////////////
/////////////////////////////////////////////////

const bool debugPython = 0;
const bool debug = 1;
const bool debugMotor = 0;
const bool debugFrein = 1;
const bool debugCapteur = 0;
const bool debugOther =1;
const bool old = 0;


/////////////////////////////////////////////////
/////////////// Frein à inertie /////////////////
/////////////////////////////////////////////////


const int frein = debugFrein?4:5;
Chrono chronoFrein;
bool freinFlag = 0;
int t1 = 300;
int t2 = 1000;
int t3 = 1500;




/////////////////////////////////////////////////
/////////////// Moteur //////////////////////////
/////////////////////////////////////////////////

MoteurEBike moteur = MoteurEBike();

/////////////////////////////////////////////////
///////////////// PID ///////////////////////////
/////////////////////////////////////////////////

double Kp = 6.5, Ki = 3.25, Kd = 0.1;

double sortieMoteur;    //output
//double valeurCapteur;   //input (valeurCapteur_ mais = 0 si < à un seuil
double consigneCapteur = 0; //setpoint
float pwmMin = 100, pwmMax = 230;
Chrono resetPID;

PID myPID(&valeurCapteur, &sortieMoteur, &consigneCapteur, Kp, Ki, Kd, REVERSE);



/////////////////////////////////////////////////
//////////// Vitesse moyenne ///////////////////
/////////////////////////////////////////////////

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

int ctrlAlive = 12;
int ctrlSwitch = 13;
bool isCtrlAlive = debugMotor?1:0;


float test = 0;
float test2 = 100;

/////////////////////////////////////////////////
///////////////// LED  //////////////////////////
/////////////////////////////////////////////////

float maxTraction = 10.0;
float seuilTractionNulle = 0.3;

RemorqueLed led = RemorqueLed(maxTraction, seuilTractionNulle, pwmMin, pwmMax);


/////////////////////////////////////////////////
///////////////// Wattmetre /////////////////////
/////////////////////////////////////////////////

Wattmeter wattmetre;

int amPin = old?A4:A6;    // select the input pin for the potentiometer  vert
int vPin = old?A5:A7;    // select the input pin for the potentiometer   jaune
float amCalib = 28.84;
float vCalib = 22.41;

bool isFlowing;
Chrono flowingChrono;
Chrono stoppedChrono;

/////////////////////////////////////////////////
//////////////// Etats //////////////////////////
/////////////////////////////////////////////////

enum etats_enum {INITIALISATION,  //0
                 ATTENTE,         //1
                 ROULE,           //2
                 STATU_QUO,       //3
                 DECCELERATION,   //4
                 FREINAGE,        //5
                 MARCHE           //6
                };

int etat = INITIALISATION;


float alpha = 1.5;  //seuil au dessus duquel le PID se calcule et se lance
float beta = -3.0;  //seuil en deça duquel on passe sur déccélération (pwm=0, pid manual)
float gamma = -6.0; // seuil en deça duquel on passe sur du freinage


int powerPin = old?0:A3; // pin pour allumer le controleur            Jaune
int motorBrakePin = old?0:A4; // pin pour activer le mode freinage    Vert
int walkPin = old?0:A5; // pin pour activer le mode piéton;           Rouge

bool powerCtrl = old?1:0;
bool walkMode = 0;
bool motorBrakeMode = 0;


void setup()
{
  Serial.begin(9600);

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
  myPID.SetSampleTime(100);

  // Controleur
  pinMode(ctrlAlive, INPUT);
  pinMode(ctrlSwitch, OUTPUT);
  digitalWrite(ctrlSwitch, false);
  attachPCINT(digitalPinToPCINT(ctrlAlive),interruptCtrlAlive,CHANGE);

  
  // Wattmetre
  wattmetre = Wattmeter(amPin,vPin,amCalib,vCalib,36);

  flowingChrono.restart();
  flowingChrono.stop();
  stoppedChrono.restart();
  stoppedChrono.stop();
     
  led.ledWelcome();

  //capteur
  capteur.setSamplesInUse(4); //16 le défaut

  pinMode(powerPin,INPUT_PULLUP);
  pinMode(motorBrakePin,INPUT_PULLUP);
  pinMode(walkPin,INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(powerPin), powerPinInterrupt, CHANGE);
  attachPCINT(digitalPinToPCINT(motorBrakePin), motorBrakePinInterrupt, CHANGE);
  attachPCINT(digitalPinToPCINT(walkPin), walkPinInterrupt, CHANGE);


}



void loop()
{
 // checkCtrl();
  miseAJourVitesse();
  if(capteurInitialise)
    capteur.update(&newDataReady,&valeurCapteur);

  wattmetre.update();

////////////////////////////////////////////////////:
///////////////////  0  ////////////////////////////
////////////////////////////////////////////////////:

    if(etat == INITIALISATION){
      Serial.println("ah les filles ah les filles");
      //switchCtrl(false);
      led.ledState(etat);
      moteur.setMoteurState(STOPPED);
      capteur.setThresholdSensor(0.5);
      switchCtrl(powerCtrl);
      if(transition5()){
        etat = FREINAGE;
      }
      else if(transition01()){
        etat = ATTENTE;
      }
    }
////////////////////////////////////////////////////:
///////////////////  1  ////////////////////////////
////////////////////////////////////////////////////:

    else if( etat == ATTENTE){

      myPID.SetMode(MANUAL); 
      led.ledState(etat);
      moteur.setMoteurState(STOPPED);
      capteur.setThresholdSensor(0.5);
      if(transition5()){
        etat = FREINAGE;
      }
      else if(transition0())
        etat = INITIALISATION;
      else if(transition12())
        etat = ROULE;
      else if(transition15())
        etat = FREINAGE;
    }

////////////////////////////////////////////////////:
///////////////////  2  ////////////////////////////
////////////////////////////////////////////////////:

    else if(etat == ROULE){
      moteur.setMoteurState(SPINNING);
      myPID.SetMode(AUTOMATIC);
      miseAJourPID();
      /*
      if(wattmetre.getState()==3)
        led.ledPrint(valeurCapteur,sortieMoteur);
      else{
        led.ledFail(2);
      }
      */
      led.ledPrint(valeurCapteur,sortieMoteur);
      capteur.setThresholdSensor(0.0);
      flowingOrNot();
      
      
      if(transition5()){
        etat = FREINAGE;
      }
      else if(transition0())
        etat = INITIALISATION;
    //  else if(transition23())
    //    etat = STATU_QUO;
    }

////////////////////////////////////////////////////:
///////////////////  3  ////////////////////////////
////////////////////////////////////////////////////:

    else if ( etat == STATU_QUO){

      led.ledState(etat);
  /*
      if(wattmetre.getState()==3)
        led.ledPrint(valeurCapteur,sortieMoteur);
      else{
        led.ledFail(3);
      }
      */
      led.ledPrint(valeurCapteur,sortieMoteur);
      myPID.SetMode(MANUAL);
      flowingOrNot();

      
      if(transition5()){
        etat = FREINAGE;
      }
      else if(transition0())
        etat = INITIALISATION;
      else  if(transition32())
        etat=ROULE;
      else if(transition34())
        etat = DECCELERATION;
    }

////////////////////////////////////////////////////:
///////////////////  4  ////////////////////////////
////////////////////////////////////////////////////:

    else if( etat == DECCELERATION){

      led.ledState(etat);
    //  moteur.setMoteurState(STOPPED);
    //  sortieMoteur=pwmMin;
      myPID.SetMode(AUTOMATIC);
      miseAJourPID();
      
      if(transition5()){
        etat = FREINAGE;
      }
      else if(transition0())
        etat = INITIALISATION;
      else  if(transition42())
        etat = ROULE;
      else if(transition45())
        etat = FREINAGE;
    }


////////////////////////////////////////////////////:
///////////////////  5  ////////////////////////////
////////////////////////////////////////////////////:

    else if(etat == FREINAGE){

      led.ledState(etat);
      myPID.SetMode(MANUAL);
      moteur.setMoteurState(BRAKING);
      sortieMoteur=pwmMin;
      capteur.setThresholdSensor(0.5);
      
      if(transition5()){
        etat = FREINAGE;
      }
      else if(transition0())
        etat = INITIALISATION;
      else   if(transition52())
        etat = ROULE;
      else if(transition51())
        etat = ATTENTE;
    }

    else{
      Serial.println("Sortie de cas");
      led.ledFail(OTHER);
    }


  if (debugPython) {

    envoi(Kd);
    envoi(sortieMoteur);
    envoi(valeurCapteur);
    envoiInt(etat);
   // envoiChrono(chronoFrein.elapsed());
    envoiFin();
  }
  else{
    debugMessage();
  //  debugTransition();
  //  Serial.println();
  //  Serial.println();

  }
  
  moteur.mettreLesGaz(sortieMoteur);


}


bool transition01(){
  //Serial.print("transition 01"); Serial.print(" - "); Serial.print(etat == INITIALISATION); Serial.print(" - "); Serial.print(initialisationCapteur()); Serial.print(" - "); Serial.print(vitesseMoyenne < 1.0); Serial.print(" - "); Serial.print(isCtrlAlive);
  //Serial.println();
  return (etat == INITIALISATION && initialisationCapteur() && vitesseMoyenne < 1.0 && isCtrlAlive);
}
bool transition12(){
  return (etat == ATTENTE && valeurCapteur>alpha && isCtrlAlive);
}
bool transition23(){
  return (etat == ROULE && (valeurCapteur<0.5 || chronoFrein.elapsed()>t1) && vitesseMoyenne > 0 && isCtrlAlive);
}
bool transition32(){
  return (etat == STATU_QUO && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition34(){
  return (etat == STATU_QUO && (valeurCapteur<beta && valeurCapteur>gamma ) || chronoFrein.elapsed()>t2 && isCtrlAlive);
}
bool transition42(){
  return (etat == DECCELERATION && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition45(){
  return (etat == DECCELERATION && valeurCapteur < gamma || chronoFrein.elapsed()>t3 && isCtrlAlive);
}
bool transition52(){
  return (etat == FREINAGE && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive && !motorBrakeMode);
}
bool transition51(){
  return (etat == FREINAGE && !chronoFrein.isRunning() && vitesseMoyenne < 1.0  && valeurCapteur >= 0.0 && valeurCapteur < alpha && isCtrlAlive  && !motorBrakeMode);
}
bool transition15(){
  return (etat == ATTENTE && valeurCapteur < gamma || chronoFrein.elapsed()>t1 && isCtrlAlive);
}
bool transition0(){
  return (!isCtrlAlive || !initialisationCapteur());
}

bool transition5(){
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
  if(digitalRead(ctrlAlive))
    isCtrlAlive=1;
  else
    isCtrlAlive=0;
}


void freinage() {
  if(debugFrein){
      if (!digitalRead(frein) && !chronoFrein.isRunning()) {
        chronoFrein.start();
      }
      else if (digitalRead(frein) && chronoFrein.isRunning()) {
        chronoFrein.restart();
        chronoFrein.stop();
      }
    
      if(!digitalRead(frein)){
        vitesseMoyenne=vitesseMoyenne?0.0:10.0;
        }
  }
  else{
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
  if((debugCapteur || debugOther) && !digitalRead(plus))
    Kd+=0.1;
}
void moinss() {
  if((debugCapteur || debugOther) && !digitalRead(moins))
    Kd-=0.1;
}
void haltt() {
 // if(!digitalRead(halt)){
 //   vitesseMoyenne=vitesseMoyenne?0.0:10.0;
 // }
}


void powerPinInterrupt(){
  if(digitalRead(powerPin)){
    powerCtrl=1;
  }
  else
    powerCtrl=0;

  switchCtrl(powerCtrl);
}
void motorBrakePinInterrupt(){
  if(digitalRead(motorBrakePin)){
    motorBrakeMode=1;
  }
  else
    motorBrakeMode=0;
}
void walkPinInterrupt(){
  if(digitalRead(walkPin))
    walkMode=1;
  else
    walkMode=0;

  setPIDMode(walkMode);
}


/*
   Fonctions annexes
*/

void setPIDMode(bool walkOrNot){
  if(walkOrNot){
    Kp = 6.5;
    Ki = 3.25;
    Kd = 0.1;
  }
  else{
    Kp = 12;
    Ki = 6;
    Kd = 0.1;
  }
}

int initialisationCapteur(){  
  if(!capteurInitialise){
    if(debugCapteur){
      capteurInitialise = 1;
      return 1;
    }
    else{
      //capteur
      capteur.begin();
      if(capteur.start()){
        Serial.println("Initialisation du capteur réussie");
        capteurInitialise = 1;
        return 1;
      }
      else{
        Serial.println("Initialisation du capteur échouée. Vérifier connexion");
        capteurInitialise = 0;  
        return 0;
      }
    }
  }
  else return 1;
}

void flowingOrNot(){ 

  /*
   * Pas bon, si ça bloque dès le démarrageet qu'on passe pas en Flowing
   */
   Serial.println("flowingOrNot");
  // if(walkMode){
    if(wattmetre.getState() == 3  && sortieMoteur>115){
      isFlowing=1;
      Serial.println("moteur tourne, rendben");
      stoppedChrono.stop();
    }
    else if(sortieMoteur>115 && wattmetre.getState() ==1){
      led.ledFail(etat);
      isFlowing=0;
      if(!stoppedChrono.isRunning())
        stoppedChrono.restart();
      else if(stoppedChrono.elapsed()>500){
        sortieMoteur=0;
        Serial.print("freeze, don't move");
       // delay(10);
        stoppedChrono.restart();
      }
   }
   else{
    Serial.println("autre cas, batterie viide?");
   }
}

void checkCtrl(){
  interruptCtrlAlive();
}

void switchCtrl(bool value){
  if(!debugMotor){
  if(value != isCtrlAlive)
    digitalWrite(ctrlSwitch,value);
  }
}

void resetCtrl(){
  if(!isCtrlAlive){
    switchCtrl(0);
    delay(500);
    switchCtrl(1);
  }
}

void miseAJourVitesse() {
  if(debugMotor){
   // Serial.println("Oups");
  }
  else{
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
//  Serial.print(" vitesse :");  Serial.print(vitesseMoyenne);  Serial.print("\t");
  Serial.print("sortie Moteur :");  Serial.print(sortieMoteur);  Serial.print("\t");
  Serial.print("Capteur :");  Serial.print(valeurCapteur);  Serial.print("\t");
//  Serial.print("Moteur state :");  Serial.print(moteur.getMoteurState());  Serial.print("\t");
  Serial.print("Etat :");  Serial.print(etat);  Serial.print("\t");  
  Serial.print("chrono :");  Serial.print(chronoFrein.isRunning());  Serial.print(" : ");  Serial.print(chronoFrein.elapsed());  Serial.print("\t");
  Serial.print("isFlowing: ");Serial.print(isFlowing);  Serial.print(" : ");  Serial.print(flowingChrono.elapsed());  Serial.print("\t");
  Serial.print("stoppedChrono :"); Serial.print(stoppedChrono.elapsed());  Serial.print("\t  | ");
  Serial.print("Ampere (raw - float): ");Serial.print(wattmetre.getCurrentRaw(true));Serial.print(" - ");Serial.print(wattmetre.getCurrent());  Serial.print("A\t");
//  Serial.print(wattmetre.getTension());  Serial.print("V\t");
  Serial.print(wattmetre.getPower());  Serial.print("W\t");
  Serial.print(wattmetre.getState());  Serial.print("\t");
  Serial.print("Kd: ");Serial.print(Kd);  Serial.print("\t");
  Serial.println();
}

void debugTransition(){
  Serial.print("0->1: \t");Serial.print(transition01());Serial.print(" | ");
  Serial.print("*->0 \t");Serial.print(transition0());Serial.print(" | ");
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
