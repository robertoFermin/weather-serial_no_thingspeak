/*
  WeatherLevel.h - Weather and Level adapter library
  Copyright (c) 2019 Sentient Things, Inc.  All right reserved.
  Based on work by Rob Purser, Mathworks, Inc.
*/

// ensure this library description is only included once
#ifndef WeatherSensors_h
#define WeatherSensors_h

// include core Particle library
#include "Particle.h"
#include <Adafruit_ADS1X15.h>

// include other libraries
#include <Adafruit_AM2315.h>
#include <SparkFun_MPL3115A2.h>
#include <RunningMedian16Bit.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_Sensor.h>
#include <IoTNode.h>

typedef struct // units chosen for data size and readability
{
    //Weather
    uint32_t unixTime; //system_tick_t (uint32_t)
    uint16_t windDegrees; // 1 degree resolution is plenty
    uint16_t wind_metersph; //meters per hour
    uint8_t humid; //percent
    uint16_t airTempKx10; // Temperature in deciKelvin
    uint16_t rainmmx1000; // millimetersx1000 - resolution is 0.2794mm 0.011"
    uint16_t barometerhPa; // Could fit into smaller type if needed
    uint16_t gust_metersph; //meters per hour
    uint16_t millivolts; // voltage in mV
    //uint16_t ozone;// concentration
    //uint16_t lux; //Light level in lux


}sensorReadings_t;
extern sensorReadings_t sensorReadings;

//struct to save created TS channel Id and keys and to check "first run"
typedef struct
{
  int testCheck;
  char unitType; // U = USA, I = international
  int firmwareVersion;
  int particleTimeout;
  float latitude;
  float longitude;
}config_t;
extern config_t config;

// library interface description
class WeatherSensors
{
  // user-accessible "public" interface
  public:
    WeatherSensors() : airTempKMedian(30), relativeHumidtyMedian(30), node()
    {
      
      pinMode(AnemometerPin, INPUT_PULLUP);
      attachInterrupt(AnemometerPin, &WeatherSensors::handleAnemometerEvent, this, FALLING);

      pinMode(RainPin, INPUT_PULLUP);
      attachInterrupt(RainPin, &WeatherSensors::handleRainEvent, this, FALLING);
    }
    void handleAnemometerEvent() {
      // Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
       unsigned int timeAnemometerEvent = millis(); // grab current time

      //If there's never been an event before (first time through), then just capture it
      if(lastAnemoneterEvent != 0) {
          // Calculate time since last event
          unsigned int period = timeAnemometerEvent - lastAnemoneterEvent;
          // ignore switch-bounce glitches less than 10mS after initial edge (which implies a max windspeed of 149mph)
          if(period < 10) {
            return;
          }
          if(period < GustPeriod) {
              // If the period is the shortest (and therefore fastest windspeed) seen, capture it
              GustPeriod = period;
          }
          AnemoneterPeriodTotal += period;
          AnemoneterPeriodReadingCount++;
      }

      lastAnemoneterEvent = timeAnemometerEvent; // set up for next event
    }

    void handleRainEvent() {
      // Count rain gauge bucket tips as they occur
      // Activated by the magnet and reed switch in the rain gauge, attached to input D2
      unsigned int timeRainEvent = millis(); // grab current time

      // ignore switch-bounce glitches less than 10mS after initial edge
      if(timeRainEvent - lastRainEvent < 10) {
        return;
      }
      rainEventCount++; //Increase this minute's amount of rain
      lastRainEvent = timeRainEvent; // set up for next event
    }

    void begin(void);
    float setModeBarometer();//cambiar modo a lectura de presion atmosferica
    float readPressure();

    float getAndResetAnemometerMPH(float * gustMPH);
    float getAndResetRainInches();

    void captureWindVane();
    void captureTempHumidityPressure();
    void captureAirTempHumid();
    void capturePressure();
    //void captureLightLux();
    void captureBatteryVoltage();

    float getAndResetWindVaneDegrees();
    float getAndResetTempF();
    float getAndResetHumidityRH();
    float getAndResetPressurePascals();
    //uint16_t getAndResetLightLux();
    uint16_t getAndResetBatteryMV();

    void getAndResetAllSensors();
    

    uint16_t getAirTempKMedian();
    uint16_t getRHMedian();
    uint16_t getAndResetO3();

    String sensorReadingsToCsvUS();
    String sensorReadingsToCsvUS(sensorReadings_t readings);

  // library-accessible "private" interface
  private:
    Adafruit_AM2315 am2315;
    MPL3115A2 barom;
    RunningMedian airTempKMedian;
    RunningMedian relativeHumidtyMedian;
    IoTNode node;
    Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

    String minimiseNumericString(String ss, int n);

    // Put pin mappings to Particle microcontroller here for now as well as
    // required variables
    // Updated for Oct 2018 v2 Weather and Level board
    int RainPin = N_D2;
    volatile unsigned int rainEventCount=0;
    unsigned int lastRainEvent;
    float RainScaleInches = 0.011; // Each pulse is .011 inches of rain


    const int AnemometerPin = N_D1;
    float AnemometerScaleMPH = 1.492; // Windspeed if we got a pulse every second (i.e. 1Hz)
    volatile unsigned int AnemoneterPeriodTotal = 0;
    volatile unsigned int AnemoneterPeriodReadingCount = 0;
    volatile unsigned int GustPeriod = UINT_MAX;
    unsigned int lastAnemoneterEvent = 0;

    //int WindVanePin = A0;
    int WindVanePin = N_A2;
    float windVaneCosTotal = 0.0;
    float windVaneSinTotal = 0.0;
    unsigned int windVaneReadingCount = 0;

    float humidityRHTotal = 0.0;
    unsigned int humidityRHReadingCount = 0;
    float tempFTotal = 0.0;
    unsigned int tempFReadingCount = 0;
    float pressurePascalsTotal = 0.0;
    unsigned int pressurePascalsReadingCount = 0;
    //Light
    //uint16_t lightLux = 0;
    //unsigned int lightLuxTotal = 0;
    //unsigned int lightLuxCount = 0;
    //Battery Voltage
    float batVoltage = 0;
    float batVoltageTotal = 0;
    unsigned int batVoltageCount = 0;
    ///
    float lookupRadiansFromRaw(unsigned int analogRaw);
    //O3
    
  //ads1115 con direccion default
    
///////////////////////////////////////////////////////////////      

};

#endif
