
#include "Arduino.h"
#ifndef Wattmeter_h
#define Wattmeter_h

#include <SoftFilters.h>


typedef enum{
  DISCONNECTED = 0,
  CONNECTED = 1,
  LOW_BATTERY = 2,
  FLOWING = 3
} wattmeter_state_t;

class Wattmeter{

  public:
    Wattmeter(uint8_t amPin = A4, uint8_t vPin=A5, float amCalibration = 65.0, float vCalibration = 18.4, uint8_t batteryNominalVoltage = 36);
    wattmeter_state_t getState();
    void update();
    float getCurrent();
    float getTension();
    int getCurrentRaw(bool raw = false);
    int getTensionRaw();
    float getPower();

  protected:
    int _amPin;
    MovingAverageFilter<int, int> _amFiltre = MovingAverageFilter<int, int>(2);
    int _amMoyen=0;
    int _amValue = 0;  // variable to store the value coming from the am
    float _amFloat = 0, _amCalibration = 1;

    int _vPin;
    MovingAverageFilter<int, int> _vFiltre = MovingAverageFilter<int, int>(64);
    int _vValue = 0, _vMoyen = 0;
    float _vFloat = 0, _vCalibration = 1;
    
    float _watt=0;

    int _batteryNominalVoltage = 0;
    wattmeter_state_t _etat = 0;
	
	int _zeroCounter = 0;
};

#endif
