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
double Kp = 6.5, Ki = 3.25, Kd = 0.1;


double consigneVitesse;
double lectureVitesse;
double sortieMoteur;

PID myPID(&lectureVitesse, &sortieMoteur, &consigneVitesse, Kp, Ki, Kd, DIRECT);


//calcul vitesse moyenne
MovingAverageFilter<double, double> vitesseFiltree(4);
double vitesseMoyenne;
double vitesseInstantanee;


int plus = 3;
int moins = 4;
int halt = 5;

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
  attachPCINT(digitalPinToPCINT(plus),pluss, CHANGE);
  attachPCINT(digitalPinToPCINT(moins),moinss, CHANGE);
  attachPCINT(digitalPinToPCINT(halt),haltt, CHANGE);

  attachPCINT(digitalPinToPCINT(moteur.getUPin()),interruptU, CHANGE);
  attachPCINT(digitalPinToPCINT(moteur.getVPin()),interruptV, CHANGE);
  

  //PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(150, 255);
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
void pluss(){
  
  if(!digitalRead(plus)){
    Kp+=1;
    /*if(consigneVitesse==0)
      consigneVitesse=10;
    else if(consigneVitesse <= 25)
      consigneVitesse+=2;
 */
  }
}
void moinss(){
  if(!digitalRead(moins))// && consigneVitesse>=2)
    Kp-=1;
    //consigneVitesse-=2;
}

void haltt(){
  if(digitalRead(halt))
    consigneVitesse=10;
   else
    consigneVitesse=0;
}

void miseAJourPID()
{
  moteur.calculerVitesse();
  vitesseInstantanee = moteur.getVitesse();
  vitesseFiltree.push(&vitesseInstantanee, &vitesseMoyenne);
  lectureVitesse = vitesseMoyenne;
  //lectureVitesse = vitesseInstantanee;
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
  Serial.print("Ki :");
  Serial.print(Ki);
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

void loop()
{
  
  miseAJourPID();
  moteur.setMoteurState(SPINNING);

  envoi(moteur.getVitesse());
  envoi(consigneVitesse);
  envoi(sortieMoteur);
  envoi(5.2);
  envoi(3.6);
  envoiFin();
  //if(consigneVitesse)
  //  envoiVitesse(moteur.getVitesse());
  //  Serial.println(moteur.getVitesse());
  //debugMessage();
}
