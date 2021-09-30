/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/

#include "StrengthSensor.h";

const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin
const double capteur_offset = 842.7;

StrengthSensor capteurDeForce(HX711_dout, HX711_sck, 842.7);


double valeur;
bool newDataReady;

void setup() {
  Serial.begin(9600); delay(10);
  
  capteurDeForce.begin();
  if(capteurDeForce.start())
    Serial.println("Initialisation du capteur réussie");
  else{
    Serial.println("Initialisation du capteur échouée. Vérifier connexion");
    while(1);
  }
}

void loop() {
  capteurDeForce.update(&newDataReady,&valeur);
  if(newDataReady){
    Serial.print("raw: ");Serial.print(capteurDeForce.getRaw());
    Serial.print("Valeur: ");Serial.print(valeur);
    Serial.println();
    newDataReady=0;
  }
}
