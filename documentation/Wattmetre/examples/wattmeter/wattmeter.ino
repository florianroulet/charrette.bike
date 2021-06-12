/*
Wattmètre


  27/12/2020
  
  En utilisant un ampli op en mode ampli sans inversion et un pont diviseur de tension, il est possible de mesurer l'ampérage et le voltage.
  Et d'en déduire la puissance consommée en instantanée

  Ampèremètre
    Op amp: MCP602
            R1  =  10  kOhm
            R2  =  470 kOhm
            RShunt = 5 mOhm
            --> Gain théorique G = 1 + R2/R1 = 48

    Calibrage:
      Avec 0.97A marqué sur le wattmètre du commerce, sur A4 en lecture analogique on a 63.
      --> Ratio de 65 u/A pour obtenir l'ampérage en ampères.


  Voltmètre
    Pont diviseur de tension
        R1 = 10 kOhm
        R2 =  1 kOhm

        U2 = U*R2/(R1+R2)

    Calibrage:
        39.58V sur le wattmètre, sur A5 en lecture analogique donne 728.
        --> Ratio de 18,4 u/V pour obtenir la tension en volts.


  Remarque:
    On gagnerait en précision à utiliser des résistances précises au 1% et non au 5% comme celles utilisées aujourd'hui.
    Il faudra alors étalonner mieux.
*/

#include <SoftFilters.h>
#include "wattmeter.h";
#include "RemorqueLed.h"

bool old = 0;
int amPin = old?A4:A6;    // select the input pin for the potentiometer  vert
int vPin = old?A5:A7;    // select the input pin for the potentiometer   jaune
float amCalib = 28.84;
float vCalib = 22.41;
char myChar();

Wattmeter wattmetre;
RemorqueLed led = RemorqueLed(2.0, 0.0, 0.0, 20.0);

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(9600);
  Serial.println("Init");
  pinMode(6,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(A5,INPUT_PULLUP);
  wattmetre = Wattmeter(amPin,vPin,amCalib,vCalib,36);
  led.ledWelcome();
  analogWrite(160,6);

}

void loop() {
  // read the value from the am:
 /* 
  amValue = analogRead(amPin);
  amFiltre.push(&amValue, &amMoyen);
  amFloat = (float) amMoyen/65.0;
  if(amValue<15)
    amFloat=0.0;
  

  vValue = analogRead(vPin);
  vFiltre.push(&vValue,&vMoyen);
  vFloat = (float)vMoyen/18.4;
  if(vValue<20)
    vFloat=0;

  watt=vFloat*amFloat;
*/

 /*
  if(Serial.available()){
    myChar=Serial.read();
    Serial.println(myChar);
  
    switch(myChar){
      case 
    }
    
  }
*/
  if(wattmetre.getCurrent()==0)
     led.ledPrint(0.5,0);
  else
     led.ledPrint(1.5, wattmetre.getCurrent());

 
  digitalWrite(13,HIGH);
 // if(digitalRead(A5))
   analogWrite(6,180);
 // else
 //  analogWrite(6,0);
   
  wattmetre.update();
 
  
 // Serial.print("raw: ");Serial.print(amValue);
 // Serial.print("\t - moyenne: ");Serial.print(amMoyen);
 // Serial.print("Etat: ");
 /*
  switch(wattmetre.getState()){
    case 0:
      Serial.print("Déconnecté");
      break;
    case 1:
      Serial.print("Connecté");Serial.println();
      break;
    case 2:
      Serial.print("Batterie faible");
      break;
    case 3:
      1+1;
     // Serial.print("Ca roule");
      break;
    default:
      Serial.print("inconnu");
  }
  */
//  
//if(wattmetre.getCurrent()==0.0)
 //Serial.print("########### 0");
  Serial.print("\t - : ");Serial.print(wattmetre.getCurrent());Serial.print("A");
 // float test = (0.0187614*wattmetre.getCurrentRaw())+0.126407;
 // Serial.print("\t - ou:  ");Serial.print(test);Serial.print("A");
  Serial.print("\t - : ");Serial.print(wattmetre.getCurrentRaw());
  Serial.print("\t - Analog : ");Serial.print(analogRead(amPin));
  Serial.print("\t - Volt: ");Serial.print(wattmetre.getTension());Serial.print("V");
  
 // Serial.print("\t --> : ");Serial.print(wattmetre.getPower());Serial.print("W");
  Serial.println();

}
