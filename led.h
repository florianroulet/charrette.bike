
#include "Arduino.h"
#ifndef Led_h
#define Led_h

//LEDs
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif



typedef enum
{
  TRACTION_PBLM = 0b00,  //  mauvaise initialisation de la barre de traction
  CTRL_PBLM = 0b01 //  problème de contrôleur
} error_state_t;

typedef enum
{
  INIT = 0b00,  
  SPIN = 0b01, 
  STATUQUO = 0b10,
  DECCELERATE = 0b11,
  BRAKE = 0b100
} state_machine_t;


class Led
{
  public:
    Led(float maxTraction, float seuilTractionNulle, float pwmMin, float pwmMax);
    int ledWelcome();  // affichage au début pour dire qu'on s'allume
    void ledState(state_machine_t state);
    void ledPrint(float valeurTraction, float pwm);   // pour afficher sur les deux panneaux de diode l'effort de traction et le pwm appliqué
    void ledFail(error_state_t error);   // en cas d'erreur*
    void ledBegin();
    void ledClear();
    uint16_t numPixels();
    int _ledPin = 9;
    int _numPixels = 16;    
    Adafruit_NeoPixel pixels=Adafruit_NeoPixel(_numPixels, _ledPin, NEO_GRB + NEO_KHZ800);

    float _maxTraction;
    float _seuilTractionNulle;
    float _pwmMin;
    float _pwmMax;

};

#endif
