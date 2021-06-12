#include "wattmeter.h"


Wattmeter::Wattmeter(uint8_t amPin, uint8_t vPin, float amCalibration, float vCalibration, uint8_t batteryNominalVoltage){
  _amPin = amPin;
  _vPin = vPin;
  _batteryNominalVoltage = batteryNominalVoltage;
  _amCalibration = amCalibration;
  _vCalibration = vCalibration;
 // MovingAverageFilter<int, int> vFiltre(16);
}

wattmeter_state_t Wattmeter::getState(){
  return _etat;
};

void Wattmeter::update(){
  
  _amValue = analogRead(_amPin);
  if(_amValue==0){
	_zeroCounter++;
    _amValue=0;
    _amFiltre.push(&_amValue, &_amMoyen);
    _amFloat = (float) _amMoyen/_amCalibration;
  }
  else{
	if(_amValue<10)
		_amValue=10;
	_zeroCounter=0;
    _amFiltre.push(&_amValue, &_amMoyen);
    _amFloat = (float) _amMoyen/_amCalibration;
  }
  if(_zeroCounter>5){
	_amFloat=0;
  }
 // if(_amFloat<0.2)
 //   _amFloat=0;
  

  _vValue = analogRead(_vPin);
  _vFiltre.push(&_vValue,&_vMoyen);
  _vFloat = (float)_vMoyen/_vCalibration;
  //if(_vValue<20)
  //  _vFloat=0;

  _watt=_vFloat*_amFloat;

  if(_vFloat<=5.0)
    _etat = DISCONNECTED;
  else if(_vFloat>5.0 && _vFloat<=(0.95*_batteryNominalVoltage))
    _etat = LOW_BATTERY;
  else
    _etat = CONNECTED;

  if(_etat == CONNECTED && _amFloat>0)
    _etat = FLOWING;
};

float Wattmeter::getCurrent(){
  return _amFloat;
};

float Wattmeter::getTension(){
  return _vFloat;
};


int Wattmeter::getCurrentRaw(bool realRaw){
	if(realRaw)
		return _amValue;
	else
  		return _amMoyen;
};

int Wattmeter::getTensionRaw(){
  return _vMoyen;
};

float Wattmeter::getPower(){
  return _watt;
};
