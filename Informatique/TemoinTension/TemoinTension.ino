/*


    Courbe d'interpolation linéaire: 16.225*U-1.17 = Analog
    42V limite basse --> 680 Analog
    54.6 limite haute --> 884

    Si analog>870: Plein, diode fixe
    Echelon entre 870 et 690 plus c'est bas, plus la période de clignotement augmente

*/

#include <Arduino.h>
#include <SoftFilters.h>

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 6;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

const int highValue=870.;
const int lowValue=690;
float diffValue=float(highValue-lowValue);

MovingAverageFilter<int, int> movAvg(5);
int avg;

int niveau=0;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

/*
 * Renverra 10 sir c'est presque 100%
 * Renverra 1 si c'est 10%
 */
int calculerNiveau(int tensionAnalogique)
{
  
  return abs(10-round(((highValue-tensionAnalogique)/diffValue*10)));
}


void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  movAvg.push(&sensorValue, &avg);
  niveau=calculerNiveau(avg);
  Serial.print("niveau: ");Serial.print(niveau);

  if(avg>=highValue)
    digitalWrite(ledPin,HIGH);
  else if(avg<highValue && avg>=lowValue){
    Serial.println(" entre les deux");
    for(int i=0; i<niveau;i++){
      digitalWrite(ledPin,HIGH);
      delay(200);
      digitalWrite(ledPin,LOW);
      delay(200);
    }
    delay(1000);
  }
  else if(avg<lowValue){
    Serial.println(" tension basse");
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(500);
    digitalWrite(ledPin,HIGH);
    delay(500);
    digitalWrite(ledPin,LOW);
    delay(500);
    digitalWrite(ledPin,HIGH);
    delay(500);
    digitalWrite(ledPin,LOW);
    delay(500);
    digitalWrite(ledPin,HIGH);
    delay(500);
    digitalWrite(ledPin,LOW);
    delay(500);
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(1000);
  }
  else{
    Serial.println(" ???");
    digitalWrite(ledPin,HIGH);
    delay(100);
    digitalWrite(ledPin,LOW);
    delay(100);
    digitalWrite(ledPin,HIGH);
    delay(500);
    digitalWrite(ledPin,LOW);
    delay(500);
  }
}
