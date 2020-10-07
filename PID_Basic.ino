/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <Chrono.h> 
#include "moteur.h"

#include <SoftFilters.h>

#include "PinChangeInterrupt.h"

/*
// PID

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
*/

/////////////////////////////////////////////////
///////////////// Chrono ////////////////////////
/////////////////////////////////////////////////

// Instanciate a Chrono object.
Chrono myChrono; 


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
//double Kp = 8, Ki = 0.2, Kd = 0.1;
double Kp = 0.8, Ki = 10, Kd = 0.1;
//double Kp = 9.5, Ki = 0, Kd = ;


double consigneVitesse;
double lectureVitesse;
double sortieMoteur;

PID myPID(&lectureVitesse, &sortieMoteur, &consigneVitesse, Kp, Ki, Kd, DIRECT);


//calcul vitesse moyenne
MovingAverageFilter<double, double> vitesseFiltree(4);
double vitesseMoyenne;
double vitesseInstantanee;


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
  
  attachPCINT(digitalPinToPCINT(moteur.getUPin()),interruptU, CHANGE);
  attachPCINT(digitalPinToPCINT(moteur.getVPin()),interruptV, CHANGE);
  

  //PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(70, 255);
  myPID.SetSampleTime(1);
  consigneVitesse = 10;
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

void miseAJourPID()
{
  moteur.calculerVitesse();
  vitesseInstantanee = moteur.getVitesse();
  vitesseFiltree.push(&vitesseInstantanee, &vitesseMoyenne);
  //lectureVitesse = vitesseMoyenne;
  lectureVitesse = vitesseInstantanee;
  //myPID.Compute();
  moteur.mettreLesGaz(80);
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
  Serial.println();
}

void envoiVitesse(float vitesse){
  byte * b = (byte *) &vitesse;
  Serial.write(b,4);
}

void loop()
{
  
  miseAJourPID();
  moteur.setMoteurState(SPINNING);

  envoiVitesse(moteur.getVitesse());
 
  //Serial.println(moteur.getVitesse());
  //debugMessage();
}
