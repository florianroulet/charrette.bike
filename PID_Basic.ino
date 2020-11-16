/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <Chrono.h> 
#include "moteur.h"
#include <SoftFilters.h>
#include <HX711_ADC.h>
#include "PinChangeInterrupt.h"

//LEDs
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


#define LED_PIN 9
#define NUMPIXELS 16
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


// Capteur
const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck); 
float THRESHOLD_CAPTEUR = 0.5; 
float THRESHOLD_CAPTEUR_STOP = -2.0; 
bool newDataReady = 0;


// frein à inertie
const int frein = 2;
Chrono chronoFrein; 
bool freinFlag = 0;


// debug
const bool debugPython = 1;

/////////////////////////////////////////////////
/////////////// Moteur //////////////////////////
/////////////////////////////////////////////////

Moteur moteur = Moteur();

/////////////////////////////////////////////////
///////////////// PID ///////////////////////////
/////////////////////////////////////////////////



//https://en.wikipedia.org/wiki/PID_controller
//Ku a vide = 15
//Tu a vide = 1,675s
// -> Kp = 0.6*Ku
// -> Ki = 0.5*Ku/Tu
// -> Kd = 3*Ku*Tu/40

//10 4.5 1.88

#define Ku 15
#define Tu 1.5
double Kp = 6.5, Ki = 3.25, Kd = 0.1;
//double Kp = 3, Ki = 0, Kd = 0;


double consigneVitesse;
double lectureVitesse;
double sortieMoteur;
double valeurCapteur, valeurCapteurSeuil;
double consigneCapteur = 0;
float pwmMin=100, pwmMax=255;
Chrono resetPID;

PID myPID(&valeurCapteurSeuil, &sortieMoteur, &consigneCapteur, Kp, Ki, Kd, REVERSE);


//calcul vitesse moyenne
MovingAverageFilter<double, double> vitesseFiltree(4);
double vitesseMoyenne;
double vitesseInstantanee;


int plus = 3;
int moins = 4;
int halt = 5;
bool haltFlag=0;

/////////////////////////////////////////////////
//////////////////// Graph //////////////////////
/////////////////////////////////////////////////

float test = 23.14;

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
  //moteur
  
  pinMode(plus, INPUT_PULLUP);
  pinMode(moins, INPUT_PULLUP);
  pinMode(halt, INPUT_PULLUP);
  pinMode(frein, INPUT_PULLUP);
  chronoFrein.stop();
  attachPCINT(digitalPinToPCINT(frein),freinage, CHANGE);
  attachPCINT(digitalPinToPCINT(plus),pluss, CHANGE);
  attachPCINT(digitalPinToPCINT(moins),moinss, CHANGE);
  attachPCINT(digitalPinToPCINT(halt),haltt, CHANGE);

  attachPCINT(digitalPinToPCINT(moteur.getUPin()),interruptU, CHANGE);
  attachPCINT(digitalPinToPCINT(moteur.getVPin()),interruptV, CHANGE);
  

  //PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(pwmMin, pwmMax);
  myPID.SetSampleTime(100);
  consigneVitesse = 10;


  pixels.begin();
  //capteur
   LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 10174.09; // uncomment this if you want to set the calibration value in the sketch
  
  long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    ledFail();
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
    ledWelcome();
  }
}

/*
 * interruption Hall
 */

void interruptU()
{
  moteur.interruptU();
}

void interruptV()
{
  moteur.interruptV();
}

void freinage(){
  if(!digitalRead(frein))
    freinFlag=1;
  else
    freinFlag=0;
}
void pluss(){
  
  if(!digitalRead(plus)){
    if(consigneVitesse==0)
      consigneVitesse=10;
    else if(consigneVitesse <= 25)
      consigneVitesse+=2;
 
  }
}
void moinss(){
  if(!digitalRead(moins) && consigneVitesse>=2)
    consigneVitesse-=2;
}
/*
void pluss(){
  
  if(!digitalRead(plus)){
    Kp+=1;
  }
}
void moinss(){
  if(!digitalRead(moins))
    Kp-=1;
}
*/
void haltt(){
  if(!digitalRead(halt)){
    consigneVitesse=10;
    haltFlag=1;
    sortieMoteur=0;
  }
   else{
    haltFlag=0;
   }
}

void miseAJourPID()
{
  moteur.calculerVitesse();
  vitesseInstantanee = moteur.getVitesse();
  vitesseFiltree.push(&vitesseInstantanee, &vitesseMoyenne);
  lectureVitesse = vitesseMoyenne;
  //lectureVitesse = vitesseInstantanee;
  if(!haltFlag)
    myPID.Compute();
  
  moteur.mettreLesGaz(sortieMoteur);
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
  Serial.print("consigne vitesse :");
  Serial.print(consigneVitesse);
  Serial.print("\t");
  Serial.print("sortie Moteur :");
  Serial.print(sortieMoteur);
  Serial.print("\t");
  Serial.print("Capteur :");
  Serial.print(valeurCapteur);
  Serial.print("\t");
  Serial.print("Capteur seuil :");
  Serial.print(valeurCapteurSeuil);
  Serial.print("\t");
  Serial.print("Moteur state :");
  Serial.print(moteur.getMoteurState());
  Serial.print("\t");
  Serial.println();
}

void envoi(float vitesse){
  byte * b = (byte *) &vitesse;
  Serial.write(b,4);
}

void envoiFin(){
  Serial.write("\n");
}


void updateCell(HX711_ADC &cell, bool &newDataReady, double &valeur){
  // check for new data/start next conversion:
  if (cell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
      float i = cell.getData();
      valeur = i;
  }
}



void ledWelcome()
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.clear();
    pixels.setPixelColor(i, pixels.Color(0, 0, 3));
    pixels.show();
    delay(30);
  }
  for (int i = NUMPIXELS; i > -1; i--)
  {
    pixels.clear();
    pixels.setPixelColor(i, pixels.Color(0, 0, 3));
    pixels.show();
    delay(30);
  }
}


void ledPrint(float valeur, float maximum, float seuilNul, float pwm, float pwmMin, float pwmMax){
  pixels.clear();
  // capteur
  int pixNb = abs(valeur/maximum)*NUMPIXELS/2;
  if(seuilNul - abs(valeur) > 0){
    // pas de mouvement
    pixels.setPixelColor(3,pixels.Color(0,0,30));
    pixels.setPixelColor(4,pixels.Color(0,0,30));
  }
  else if(valeur>0){
    // traction
    for(int i = 0; i< pixNb; i++){
       pixels.setPixelColor(i,pixels.Color(0,30,0));
    }
  }
  else{
    // compression
    for(int i = 0; i< pixNb; i++){
       pixels.setPixelColor(i,pixels.Color(30,0,0));
    }
  }

  // pwm
  if(moteur.getMoteurState()!=STOPPED){
    int pwmPixNb = (pwm-pwmMin)/(pwmMax-pwmMin)*(NUMPIXELS/2)+NUMPIXELS/2;
    if(pwm>pwmMin){
      for(int i = NUMPIXELS/2; i< pwmPixNb; i++){
         pixels.setPixelColor(i,pixels.Color(0,30,30));
      }
    }
    else{
      for(int i = NUMPIXELS/2; i< NUMPIXELS; i++){
         pixels.setPixelColor(i,pixels.Color(30,30,0));
     }
    }
  }
  else{
     for(int i = NUMPIXELS/2; i< NUMPIXELS; i++){
         pixels.setPixelColor(i,pixels.Color(30,0,30));
     }
  }
  
  pixels.show();
}

void ledFail(){
  pixels.clear();
  for(int i = 2; i< NUMPIXELS-2; i++){
     pixels.setPixelColor(i,pixels.Color(30,30,30));
  }
  pixels.show();
}

void loop()
{
  
  updateCell(LoadCell, newDataReady, valeurCapteur);
 /*
  if((valeurCapteur<THRESHOLD_CAPTEUR) && valeurCapteur>0){
    valeurCapteurSeuil = 0;
    myPID.SetMode(MANUAL);
    //sortieMoteur=pwmMin;
  }
  else{
    myPID.SetMode(AUTOMATIC);
    valeurCapteurSeuil = valeurCapteur;
  }
  */
  if((valeurCapteur<THRESHOLD_CAPTEUR) && valeurCapteur>0){
    valeurCapteurSeuil = 0;
    //sortieMoteur=pwmMin;
  }
  else{
    valeurCapteurSeuil = valeurCapteur; 
  }
  if(valeurCapteur<THRESHOLD_CAPTEUR_STOP){
    moteur.setMoteurState(STOPPED);
    valeurCapteurSeuil = 0;
    myPID.SetMode(MANUAL);
    sortieMoteur=pwmMin;
  }
  else{
    myPID.SetMode(AUTOMATIC);
  }
    
  miseAJourPID();
  if(lectureVitesse<2 && valeurCapteurSeuil <= 0)
    moteur.setMoteurState(STOPPED);
  else
    moteur.setMoteurState(SPINNING);
  if(newDataReady){
    //rafraichissement toutes les 100ms environ
    newDataReady=0;
    ledPrint(valeurCapteur,10.0,0.3,sortieMoteur,pwmMin,pwmMax);
  }



  
  if(debugPython){
  
    envoi(moteur.getVitesse());
    envoi(consigneVitesse);
    envoi(sortieMoteur);
    envoi(valeurCapteur);
    envoiFin();
  }
  else
    debugMessage();
  
 
}
