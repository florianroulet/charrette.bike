#include <Chrono.h>
#include "moteur.h"
#include <PID_v1.h>
#include "led.h"
#include <SoftFilters.h>
#include <HX711_ADC.h>
#include "PinChangeInterrupt.h"





/////////////////////////////////////////////////
/////////////// capteur force ///////////////////
/////////////////////////////////////////////////

const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
double valeurCapteur_;
float THRESHOLD_CAPTEUR = 0.5;
float THRESHOLD_CAPTEUR_STOP = -2.0;
bool newDataReady = 0;
bool capteurInitialise = 0;


/////////////////////////////////////////////////
/////////////// Debug ///////////////////////////
/////////////////////////////////////////////////

const bool debugPython = 0;
const bool debug = 1;
const bool debugMotor = 0;


/////////////////////////////////////////////////
/////////////// Frein à inertie /////////////////
/////////////////////////////////////////////////


const int frein = debug?5:2;
Chrono chronoFrein;
bool freinFlag = 0;
int t1 = 300;
int t2 = 1000;
int t3 = 1500;




/////////////////////////////////////////////////
/////////////// Moteur //////////////////////////
/////////////////////////////////////////////////

Moteur moteur = Moteur();

/////////////////////////////////////////////////
///////////////// PID ///////////////////////////
/////////////////////////////////////////////////

double Kp = 6.5, Ki = 3.25, Kd = 0.1;

double sortieMoteur;    //output
double valeurCapteur;   //input (valeurCapteur_ mais = 0 si < à un seuil
double consigneCapteur = 0; //setpoint
float pwmMin = 100, pwmMax = 255;
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

int plus = 3;
int moins = 4;
int halt = 5;
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

Led led = Led(maxTraction, seuilTractionNulle, pwmMin, pwmMax);


/////////////////////////////////////////////////
//////////////////// Graph //////////////////////
/////////////////////////////////////////////////


/////////////////////////////////////////////////
//////////////// Etats //////////////////////////
/////////////////////////////////////////////////

enum etats_enum {INITIALISATION,
                 ATTENTE,
                 ROULE,
                 STATU_QUO,
                 DECCELERATION,
                 FREINAGE
                };

uint8_t etat = INITIALISATION;


float alpha = 1.0;
float beta = -2.0;
float gamma = -5.0;


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

  
  
  led.ledWelcome();


}



void loop()
{
  miseAJourVitesse();
  if(capteurInitialise)
    updateCell(LoadCell, newDataReady, valeurCapteur_);


    // etat 0
    if(etat == INITIALISATION){
      switchCtrl(false);
      led.ledState(etat);
      moteur.setMoteurState(STOPPED);
      switchCtrl(true);
      if(transition01()){
        etat = ATTENTE;
      }
    }

    // etat 1
    else if( etat == ATTENTE){

      myPID.SetMode(MANUAL); 
      led.ledState(etat);
      moteur.setMoteurState(STOPPED);
      if(transition0())
        etat = INITIALISATION;
      else if(transition12())
        etat = ROULE;
      else if(transition15())
        etat = FREINAGE;
    }

    //etat 2
    else if(etat == ROULE){

      moteur.setMoteurState(SPINNING);
      myPID.SetMode(AUTOMATIC);
      miseAJourPID();
      led.ledPrint(valeurCapteur,sortieMoteur);
      if(transition0())
        etat = INITIALISATION;
      else if(transition23())
        etat = STATU_QUO;
    }

    //etat 3
    else if ( etat == STATU_QUO){

      led.ledState(etat);
      myPID.SetMode(MANUAL);
      if(transition0())
        etat = INITIALISATION;
      else  if(transition32())
        etat=ROULE;
      else if(transition34())
        etat = DECCELERATION;
    }

    //etat 4
    else if( etat == DECCELERATION){

      led.ledState(etat);
      moteur.setMoteurState(STOPPED);
      sortieMoteur=pwmMin;
      if(transition0())
        etat = INITIALISATION;
      else  if(transition42())
        etat = ROULE;
      else if(transition45())
        etat = FREINAGE;
    }

    //etat 5
    else if(etat == FREINAGE){

      led.ledState(etat);
      moteur.setMoteurState(BRAKING);
      if(transition0())
        etat = INITIALISATION;
      else   if(transition52())
        etat = ROULE;
      else if(transition51())
        etat = ATTENTE;
    }

    else{

      led.ledFail(OTHER);
    }


  if (debugPython) {

    envoi(moteur.getVitesse());
    envoi(sortieMoteur);
    envoi(valeurCapteur);
    envoiFin();
  }
  else{
    debugMessage();
    debugTransition();
    Serial.println();
    Serial.println();

  }


  moteur.mettreLesGaz(sortieMoteur);


}


bool transition01(){
  return (etat == INITIALISATION && initialisationCapteur() && vitesseMoyenne == 0 && isCtrlAlive);
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
  return (etat == STATU_QUO && (valeurCapteur<0 && valeurCapteur>beta ) || chronoFrein.elapsed()>t2 && isCtrlAlive);
}
bool transition42(){
  return (etat == DECCELERATION && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition45(){
  return (etat == DECCELERATION && valeurCapteur < gamma || chronoFrein.elapsed()>t3 && isCtrlAlive);
}
bool transition52(){
  return (etat == FREINAGE && valeurCapteur > alpha && !chronoFrein.isRunning() && isCtrlAlive);
}
bool transition51(){
  return (etat == FREINAGE && !chronoFrein.isRunning() && vitesseMoyenne == 0 && valeurCapteur == 0 && isCtrlAlive);
}
bool transition15(){
  return (etat == ATTENTE && valeurCapteur < gamma || chronoFrein.elapsed()>t1 && isCtrlAlive);
}
bool transition0(){
  return (!isCtrlAlive || !initialisationCapteur());
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
    isCtrlAlive=0;
  else
    isCtrlAlive=1;
}


void freinage() {
  if (!digitalRead(frein) && !chronoFrein.isRunning()) {
    chronoFrein.start();
  }
  else if (digitalRead(frein) && chronoFrein.isRunning()) {
    chronoFrein.restart();
    chronoFrein.stop();
  }

  if(debugMotor){
    if(!digitalRead(frein)){
      vitesseMoyenne=vitesseMoyenne?0.0:10.0;
    }
  }
}
void pluss() {
  if(debug && !digitalRead(plus))
    valeurCapteur+=0.2;
}
void moinss() {
  if(debug && !digitalRead(moins))
    valeurCapteur-=0.2;
}
void haltt() {
  if(!digitalRead(halt)){
    vitesseMoyenne=vitesseMoyenne?0.0:10.0;
  }
}



/*
   Fonctions annexes
*/

int initialisationCapteur(){
  if(debug){
    capteurInitialise = 1;
    return 1;
  }
  else{
    //capteur
    LoadCell.begin();
    float calibrationValue; // calibration value (see example file "Calibration.ino")
    calibrationValue = 10174.09; // uncomment this if you want to set the calibration value in the sketch
  
    long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag()) {
      Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
      led.ledFail(0);
      return 0;
    }
    else {
      LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
      Serial.println("Startup is complete");
      capteurInitialise = 1;
      return 1;
    } 
  }
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
  Serial.print(" vitesse :");
  Serial.print(vitesseMoyenne);
  Serial.print("\t");
  Serial.print("sortie Moteur :");
  Serial.print(sortieMoteur);
  Serial.print("\t");
  Serial.print("Capteur :");
  Serial.print(valeurCapteur);
  Serial.print("\t");
  Serial.print("Moteur state :");
  Serial.print(moteur.getMoteurState());
  Serial.print("\t");
  Serial.print("Etat :");
  Serial.print(etat);
  Serial.print("\t");
  Serial.print("chrono :");
  Serial.print(chronoFrein.isRunning());
  Serial.print("\t");
  Serial.print(chronoFrein.elapsed());
  Serial.print("\t");
  Serial.println();
}

void debugTransition(){
  Serial.print("0->1: \t");Serial.print(transition01());Serial.print(" | ");
  Serial.print("1->2: \t");Serial.print(transition12());Serial.print(" | ");
  Serial.print("1->5: \t");Serial.print(transition15());Serial.print(" | ");
  Serial.print("2->3: \t");Serial.print(transition23());Serial.print(" | ");
  Serial.print("3->2: \t");Serial.print(transition32());Serial.print(" | ");
  Serial.print("3->4: \t");Serial.print(transition34());Serial.print(" | ");
  Serial.print("4->2: \t");Serial.print(transition42());Serial.print(" | ");
  Serial.print("4->5: \t");Serial.print(transition45());Serial.print(" | ");
  Serial.print("5->2: \t");Serial.print(transition52());Serial.print(" | ");
  Serial.print("5->1: \t");Serial.print(transition51());Serial.println("");
}

void envoi(float vitesse) {
  byte * b = (byte *) &vitesse;
  Serial.write(b, 4);
}

void envoiFin() {
  Serial.write("\n");
}


void updateCell(HX711_ADC &cell, bool &newDataReady, double &valeur) {
  if(!debug){
    // check for new data/start next conversion:
    if (cell.update()) newDataReady = true;
  
    // get smoothed value from the dataset:
    if (newDataReady) {
      float i = cell.getData();
      valeur = i;
    }  
    // si valeur capteur inférieur à un seuil mais positive, comme si c'était 0.
    if((abs(valeur)<THRESHOLD_CAPTEUR) && valeur>0){
      valeurCapteur = 0;
    }
    else{
      valeurCapteur = valeur; 
    }
  }
}
