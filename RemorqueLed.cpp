#include "RemorqueLed.h"

RemorqueLed::RemorqueLed(float maxTraction, float seuilTractionNulle, float pwmMin, float pwmMax){
  _maxTraction=maxTraction;
  _seuilTractionNulle=seuilTractionNulle;
  _pwmMin = pwmMin;
  _pwmMax = pwmMax;
  pixels.begin();  
}

void RemorqueLed::ledState(state_machine_t state){
  if(state>=0 && state <=5){
      pixels.clear();
      pixels.setPixelColor(state, pixels.Color(100, 0, 0));
      pixels.show();
  }
    else
        ledFail(OTHER);
}

void RemorqueLed::ledBegin(){
  pixels.begin();
}

uint16_t RemorqueLed::numPixels(){
  return pixels.getPin();
}

void RemorqueLed::ledClear(){
  pixels.clear();
}

int RemorqueLed::ledWelcome(){

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

void RemorqueLed::ledPrint(float traction, float pwm){
  pixels.clear();
  
  // capteur
  int pixNb = abs(traction/_maxTraction)*_numPixels/2;
  if(_seuilTractionNulle - abs(traction) > 0){
    // pas de mouvement
    if(_mode==0){
	    pixels.setPixelColor(3,pixels.Color(0,0,30));
	    pixels.setPixelColor(4,pixels.Color(0,0,30));
    }
    else{
	    pixels.setPixelColor(2,pixels.Color(0,0,30));
	    pixels.setPixelColor(3,pixels.Color(0,0,30));
	    pixels.setPixelColor(4,pixels.Color(0,0,30));
	    pixels.setPixelColor(5,pixels.Color(0,0,30));
    }
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
       if(_mode==0)
           pixels.setPixelColor(i,pixels.Color(0,30,30));
       else
           pixels.setPixelColor(i,pixels.Color(30,00,30));
     }
  }
  else{
    for(int i = _numPixels/2; i< _numPixels; i++){
       if(_mode==0)
           pixels.setPixelColor(i,pixels.Color(30,30,0));
       else
           pixels.setPixelColor(i,pixels.Color(30,00,30));
   }
  }
  
  pixels.show();

 
}

void RemorqueLed::ledWait(float traction){
  pixels.clear();
  
  // capteur
  int pixNb = abs(traction/_maxTraction)*_numPixels/2;
  if(_seuilTractionNulle - abs(traction) > 0){
    // pas de mouvement
    if(_mode==0){
      pixels.setPixelColor(3,pixels.Color(0,0,30));
      pixels.setPixelColor(4,pixels.Color(0,0,30));
    }
    else{
      pixels.setPixelColor(2,pixels.Color(0,0,30));
      pixels.setPixelColor(3,pixels.Color(0,0,30));
      pixels.setPixelColor(4,pixels.Color(0,0,30));
      pixels.setPixelColor(5,pixels.Color(0,0,30));
    }
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

      for(int i = _numPixels/2+1; i< _numPixels-1; i++){
       if(_mode==0)
           pixels.setPixelColor(i,pixels.Color(30,30,0));
       else
           pixels.setPixelColor(i,pixels.Color(30,00,30));
   }
  pixels.show();

}


void RemorqueLed::ledFail(error_state_t error){
  pixels.clear();
  pixels.setPixelColor(error,pixels.Color(100,100,0));
  pixels.show();
  
}


void RemorqueLed::setMode(int mode){
  _mode=mode;
}
