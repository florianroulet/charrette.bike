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



int amPin = A4;    // select the input pin for the potentiometer
int vPin = A5;    // select the input pin for the potentiometer
float amCalib = 65.0;
float vCalib = 18.4;

Wattmeter wattmetre;

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin(9600);
  Serial.println("Init");
  wattmetre = Wattmeter(amPin,vPin,amCalib,vPin,36);
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
  wattmetre.update();
  
  
 // Serial.print("raw: ");Serial.print(amValue);
 // Serial.print("\t - moyenne: ");Serial.print(amMoyen);
  Serial.print("Etat: ");
  switch(wattmetre.getState()){
    case 0:
      Serial.print("Déconnecté");
      break;
    case 1:
      Serial.print("Connecté");
      break;
    case 2:
      Serial.print("Batterie faible");
      break;
    case 3:
      Serial.print("Ca roule");
      break;
    default:
      Serial.print("inconnu");
  }
  Serial.print("\t - : ");Serial.print(wattmetre.getCurrent());Serial.print("A");
  Serial.print("\t - : ");Serial.print(wattmetre.getTension());Serial.print("V");
  Serial.print("\t - : ");Serial.print(wattmetre.getCurrentRaw());
  Serial.print("\t - : ");Serial.print(wattmetre.getTensionRaw());
  Serial.print("\t --> : ");Serial.print(wattmetre.getPower());Serial.print("W");
  Serial.println();
}
