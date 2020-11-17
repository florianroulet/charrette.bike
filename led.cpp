#include "led.h"

Led::Led(float maxTraction, float seuilTractionNulle, float pwmMin, float pwmMax){
  _maxTraction=maxTraction;
  _seuilTractionNulle=seuilTractionNulle;
  _pwmMin = pwmMin;
  _pwmMax = pwmMax;
  pixels.begin();  
}

void Led::ledState(state_machine_t state){
  
}

void Led::ledBegin(){
  pixels.begin();
}

uint16_t Led::numPixels(){
  return pixels.getPin();
}

void Led::ledClear(){
  pixels.clear();
}

int Led::ledWelcome(){

  pixels.clear();
  for (int i = 0; i < _numPixels; i++)
  {
    pixels.clear();
    pixels.setPixelColor(i, pixels.Color(0, 0, 3));
    pixels.show();
    delay(30);
  }
  for (int i = _numPixels; i > -1; i--)
  {
    pixels.clear();
    pixels.setPixelColor(i, pixels.Color(0, 0, 3));
    pixels.show();
    delay(30);
  }
  return 1;
  
}

void Led::ledPrint(float traction, float pwm){
  pixels.clear();
  
  // capteur
  int pixNb = abs(traction/_maxTraction)*_numPixels/2;
  if(_seuilTractionNulle - abs(traction) > 0){
    // pas de mouvement
    pixels.setPixelColor(3,pixels.Color(0,0,30));
    pixels.setPixelColor(4,pixels.Color(0,0,30));
  }
  else if(traction>0){
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


  //PWM
  int pwmPixNb = (pwm - _pwmMin)/(_pwmMax - _pwmMin)*(_numPixels/2)+_numPixels/2;
  if(pwm>_pwmMin){
    for(int i = _numPixels/2; i< pwmPixNb; i++){
       pixels.setPixelColor(i,pixels.Color(0,30,30));
    }
  }
  else{
    for(int i = _numPixels/2; i< _numPixels; i++){
       pixels.setPixelColor(i,pixels.Color(30,30,0));
   }
  }
  
  pixels.show();

 
}

void Led::ledFail(error_state_t error){
  pixels.clear();
  for(int i = 2; i< _numPixels-2; i++){
     pixels.setPixelColor(i,pixels.Color(30,30,30));
  }
  pixels.show();
  
}
