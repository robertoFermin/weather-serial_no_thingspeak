


// include this library's description file
#include "WeatherSensors.h"
#include <math.h>
#include <stdio.h>


// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

/*WeatherSensors::WeatherLevel()
{
  // initialize this instance's variables


  // do whatever is required to initialize the library

}
*/
// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries
void WeatherSensors::begin(void)
{

  AnemoneterPeriodTotal = 0;
  AnemoneterPeriodReadingCount = 0;
  GustPeriod = UINT_MAX;  //  The shortest period (and therefore fastest gust) observed
  lastAnemoneterEvent = 0;


  barom.begin();
  barom.setModeBarometer();
  barom.setOversampleRate(7);
  barom.enableEventFlags();

  am2315.begin();

  Serial.begin(9600);

  Serial.println(F("Starting Adafruit TSL2591 Test!"));
  
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
  }
    
  /* Display some basic information on this sensor */
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
  
  /* Configure the sensor */
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}

float  WeatherSensors::getAndResetAnemometerMPH(float * gustMPH)
{
    if(AnemoneterPeriodReadingCount == 0)
    {
        *gustMPH = 0.0;
        return 0;
    }
    // Nonintuitive math:  We've collected the sum of the observed periods between pulses, and the number of observations.
    // Now, we calculate the average period (sum / number of readings), take the inverse and muliple by 1000 to give frequency, and then mulitply by our scale to get MPH.
    // The math below is transformed to maximize accuracy by doing all muliplications BEFORE dividing.
    float result = AnemometerScaleMPH * 1000.0 * float(AnemoneterPeriodReadingCount) / float(AnemoneterPeriodTotal);
    AnemoneterPeriodTotal = 0;
    AnemoneterPeriodReadingCount = 0;
    *gustMPH = AnemometerScaleMPH  * 1000.0 / float(GustPeriod);
    GustPeriod = UINT_MAX;
    return result;
}

void WeatherSensors::captureBatteryVoltage()
{
  unsigned int rawVoltage = 0;
  Wire.requestFrom(0x4D, 2);
  if (Wire.available() == 2)
  {
    rawVoltage = (Wire.read() << 8) | (Wire.read());
    batVoltageTotal += (float)(rawVoltage)/4096.0*13.64; // 3.3*(4.7+1.5)/1.5
    batVoltageCount ++;
  }  
}

uint16_t WeatherSensors::getAndResetBatteryMV()
{
 uint16_t result = (uint16_t) 1000*(batVoltageTotal/batVoltageCount);
 batVoltageTotal = 0;
 batVoltageCount = 0;
 return result;
}

//void WeatherSensors::captureLightLux()
//{
  //sensors_event_t event;
  //tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  // Serial.print(F("[ ")); Serial.print(event.timestamp); Serial.print(F(" ms ] "));
  //if ((event.light == 0) |
      //(event.light > 4294966000.0) | 
     // (event.light <-4294966000.0))
  //{
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    //Serial.println(F("Invalid data (adjust gain or timing)"));
  //}
  //else
  //{
    //lightLuxTotal += (unsigned int)event.light;
    //lightLuxCount ++;
    // Serial.println(lightLuxTotal);
    // Serial.println(lightLuxCount);    
    // Serial.print(event.light); Serial.println(F(" lux"));
  //}

//}

/*uint16_t WeatherSensors::getAndResetLightLux()
{
  if (lightLuxTotal)
  {
    uint16_t result = (uint16_t)(lightLuxTotal/lightLuxCount);
    lightLuxTotal=0;
    lightLuxCount=0;
    return result;
  }
  else
  {
    lightLuxTotal=0;
    lightLuxCount=0;
    return NULL;
  } 
}*/


float WeatherSensors::getAndResetRainInches()
{
    float result = RainScaleInches * float(rainEventCount);
    rainEventCount = 0;
    return result;
}
/// Wind Vane
void WeatherSensors::captureWindVane() {
    // Read the wind vane, and update the running average of the two components of the vector
    unsigned int windVaneRaw = analogRead(WindVanePin);
    //Serial.println(windVaneRaw);
    float windVaneRadians = lookupRadiansFromRaw(windVaneRaw);
    //Serial.println(windVaneRadians);
    if(windVaneRadians > 0 && windVaneRadians < 6.14159)
    {
        windVaneCosTotal += cos(windVaneRadians);
        windVaneSinTotal += sin(windVaneRadians);
        windVaneReadingCount++;
    }
    return;
}

float WeatherSensors::getAndResetWindVaneDegrees()
{
    if(windVaneReadingCount == 0) {
        return 0;
    }
    float avgCos = windVaneCosTotal/float(windVaneReadingCount);
    float avgSin = windVaneSinTotal/float(windVaneReadingCount);
    float result = atan(avgSin/avgCos) * 180.0 / 3.14159;
    windVaneCosTotal = 0.0;
    windVaneSinTotal = 0.0;
    windVaneReadingCount = 0;
    // atan can only tell where the angle is within 180 degrees.  Need to look at cos to tell which half of circle we're in
    if(avgCos < 0) result += 180.0;
    // atan will return negative angles in the NW quadrant -- push those into positive space.
    if(result < 0) result += 360.0;

   return result;
}

float WeatherSensors::lookupRadiansFromRaw(unsigned int analogRaw)
{
//Serial.println(analogRaw);
    // The mechanism for reading the weathervane isn't arbitrary, but effectively, we just need to look up which of the 16 positions we're in.
    if(analogRaw >= 2200 && analogRaw < 2400) return (3.14);//South
    if(analogRaw >= 2100 && analogRaw < 2200) return (3.53);//SSW
    if(analogRaw >= 3200 && analogRaw < 3299) return (3.93);//SW
    if(analogRaw >= 3100 && analogRaw < 3200) return (4.32);//WSW
    if(analogRaw >= 3890 && analogRaw < 3999) return (4.71);//West
    if(analogRaw >= 3700 && analogRaw < 3780) return (5.11);//WNW
    if(analogRaw >= 3780 && analogRaw < 3890) return (5.50);//NW
    if(analogRaw >= 3400 && analogRaw < 3500) return (5.89);//NNW
    if(analogRaw >= 3570 && analogRaw < 3700) return (0.00);//North
    if(analogRaw >= 2600 && analogRaw < 2700) return (0.39);//NNE
    if(analogRaw >= 2750 && analogRaw < 2850) return (0.79);//NE
    if(analogRaw >= 1510 && analogRaw < 1580) return (1.18);//ENE
    if(analogRaw >= 1580 && analogRaw < 1650) return (1.57);//East
    if(analogRaw >= 1470 && analogRaw < 1510) return (1.96);//ESE
    if(analogRaw >= 1900 && analogRaw < 2000) return (2.36);//SE
    if(analogRaw >= 1700 && analogRaw < 1750) return (2.74);//SSE
    if(analogRaw > 4000) return(-1); // Open circuit?  Probably means the sensor is not connected
   // Particle.publish("error", String::format("Got %d from Windvane.",analogRaw), 60 , PRIVATE);
    return -1;
}

/// end Wind vane

void WeatherSensors::captureTempHumidityPressure() {
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations

  // Measure Relative Humidity and temperature from the AM2315
  float humidityRH, tempC, tempF;
  bool validTH = am2315.readTemperatureAndHumidity(tempC, humidityRH);

  uint16_t tempKx10 = uint16_t(tempC*10)+2732;
  airTempKMedian.add(tempKx10);

  relativeHumidtyMedian.add(humidityRH);

if (validTH){
    //If the result is reasonable, add it to the running mean
    if(humidityRH > 0 && humidityRH < 105) // It's theoretically possible to get supersaturation humidity levels over 100%
    {
        // Add the observation to the running sum, and increment the number of observations
        humidityRHTotal += humidityRH;
        humidityRHReadingCount++;
    }


    tempF = (tempC * 9.0) / 5.0 + 32.0;
    //If the result is reasonable, add it to the running mean
    if(tempF > -50 && tempF < 150)
    {
        // Add the observation to the running sum, and increment the number of observations
        tempFTotal += tempF;
        tempFReadingCount++;
    }
  }
  //Measure Pressure from the MPL3115A2
  float pressurePascals = barom.readPressure();

  //If the result is reasonable, add it to the running mean
  // What's reasonable? http://findanswers.noaa.gov/noaa.answers/consumer/kbdetail.asp?kbid=544
  if(pressurePascals > 80000 && pressurePascals < 110000)
  {
      // Add the observation to the running sum, and increment the number of observations
      pressurePascalsTotal += pressurePascals;
      pressurePascalsReadingCount++;
  }
  return;
}

void WeatherSensors::captureAirTempHumid() {
  // Read the humidity and pressure sensors, and update the running average
  // The running (mean) average is maintained by keeping a running sum of the observations,
  // and a count of the number of observations

  // Measure Relative Humidity and temperature from the AM2315
  float humidityRH, tempC, tempF;
  bool validTH = am2315.readTemperatureAndHumidity(tempC, humidityRH);

  uint16_t tempKx10 = uint16_t(tempC*10.0+2732.0);
  airTempKMedian.add(tempKx10);

  relativeHumidtyMedian.add(humidityRH);

  if (validTH){
      //If the result is reasonable, add it to the running mean
      if(humidityRH > 0 && humidityRH < 105) // It's theoretically possible to get supersaturation humidity levels over 100%
      {
          // Add the observation to the running sum, and increment the number of observations
          humidityRHTotal += humidityRH;
          humidityRHReadingCount++;
      }


      tempF = (tempC * 9.0) / 5.0 + 32.0;
      //If the result is reasonable, add it to the running mean
      if(tempF > -50 && tempF < 150)
      {
          // Add the observation to the running sum, and increment the number of observations
          tempFTotal += tempF;
          tempFReadingCount++;
      }
    }
}

void WeatherSensors::capturePressure() {
  //Measure Pressure from the MPL3115A2
  float pressurePascals = barom.readPressure();

  //If the result is reasonable, add it to the running mean
  // What's reasonable? http://findanswers.noaa.gov/noaa.answers/consumer/kbdetail.asp?kbid=544
  if(pressurePascals > 80000 && pressurePascals < 110000)
  {
      // Add the observation to the running sum, and increment the number of observations
      pressurePascalsTotal += pressurePascals;
      pressurePascalsReadingCount++;
  }
}

float WeatherSensors::getAndResetTempF()
{
    if(tempFReadingCount == 0) {
        return 0;
    }
    float result = tempFTotal/float(tempFReadingCount);
    tempFTotal = 0.0;
    tempFReadingCount = 0;
    return result;
}

float WeatherSensors::getAndResetHumidityRH()
{
    if(humidityRHReadingCount == 0) {
        return 0;
    }
    float result = humidityRHTotal/float(humidityRHReadingCount);
    humidityRHTotal = 0.0;
    humidityRHReadingCount = 0;
    return result;
}


float WeatherSensors::getAndResetPressurePascals()
{
    if(pressurePascalsReadingCount == 0) {
        return 0;
    }
    float result = pressurePascalsTotal/float(pressurePascalsReadingCount);
    pressurePascalsTotal = 0.0;
    pressurePascalsReadingCount = 0;
    return result;
}


uint16_t WeatherSensors::getAirTempKMedian()
{
  uint16_t airKMedian = airTempKMedian.getMedian();
  return airKMedian;
}

uint16_t WeatherSensors::getRHMedian()
{
  uint16_t RHMedian = relativeHumidtyMedian.getMedian();
  return RHMedian;
}
/*uint16_t WeatherSensors::getAndResetO3(){//sensor spec de ozono

 Vvgas_o3 = (ads.readADC_SingleEnded(0)*0.1875)/1000.0;
    Vvref_o3 = (ads.readADC_SingleEnded(1)*0.1875)/1000.0; //* ADC_RESOLUTION; // read the input pin
    Vtemp_o3 = (ads.readADC_SingleEnded(2)*0.1875)/1000.0;
    tempVin = (87/3.3)*Vtemp_o3 - 18;//temperatura en centigrados
    float concentration_o3 = (Vvgas_o3 - 1.654640) / (499*1000000000) / (-33.56);//PPM
 
    tiempo1=millis();//loop para promedio de datos 
if(tiempo1 <= (tiempo2+3600000)){//1752 datos en una hora
    cont++;
    sum += concentration_o3;
    prom =sum / cont;
     //Serial.println("Ozone Concentration: " + String(prom, 15)+ "ppm/n"); //
     return prom,15;
     return tempVin;
     //Serial.println("Voltaje: "+ String(Vvgas_o3)+ "mV");
}else{
    cont=0;
    tiempo2=millis();
    sum=0;
    prom = 0;
}
}
*/
float WeatherSensors::readPressure()
{
  return barom.readPressure();
}
/*
//Weather
    uint32_t unixTime; //system_tick_t (uint32_t)
    uint16_t windDegrees; // 1 degree resolution is plenty
    uint16_t wind_metersph; //meters per hour
    uint8_t humid; //percent
    uint16_t airTempKx10; // Temperature in deciKelvin
    uint16_t rainmmx1000; // millimetersx1000 - resolution is 0.2794mm 0.011"
    float barometerhPa; // Could fit into smaller type if needed
    uint16_t gust_metersph; //meters per hour
    uint16_t millivolts; // voltage in mV
    //uint16_t lux; //Light level in lux
*/
void WeatherSensors::getAndResetAllSensors()
{
  uint32_t timeRTC = node.unixTime();
  sensorReadings.unixTime = timeRTC;
  float gustMPH;
  float windMPH = getAndResetAnemometerMPH(&gustMPH);
  sensorReadings.wind_metersph = (uint16_t) ceil(windMPH * 1609.34);
  //Particle.publish("viento en km por hora", String(sensorReadings.wind_metersph));
  float rainInches = getAndResetRainInches();
  sensorReadings.rainmmx1000 = (uint16_t) ceil(rainInches * 25400);
  //Particle.publish("pulgadas de lluvia", String(sensorReadings.rainmmx1000));
  float windDegrees = getAndResetWindVaneDegrees();
  sensorReadings.windDegrees = (uint16_t) ceil(windDegrees);
  //Particle.publish("direccion del viento" , String(sensorReadings.windDegrees));
  float airTempF = getAndResetTempF();
  sensorReadings.airTempKx10 = (uint16_t) ceil((airTempF-32.0)*50.0/9.0 + 2731.5);
  //Particle.publish("Temperatura", String(sensorReadings.airTempKx10));
  uint16_t humidityRH = relativeHumidtyMedian.getMedian();
  sensorReadings.humid =(uint8_t) ceil(humidityRH);
   //float O3 = getAndResetO3();
  //sensorReadings.ozone=(uint16_t) ceil(O3);
  //Particle.publish("Humedad", String(sensorReadings.humid));
  float pressure = getAndResetPressurePascals();
  sensorReadings.barometerhPa = pressure/10.0;
  //Particle.publish("Presion", String(sensorReadings.barometerhPa));
  // Light and voltage needed
 // sensorReadings.lux = getAndResetLightLux();
  //Particle.publish("luminocidad", String(sensorReadings.lux));
  sensorReadings.millivolts=getAndResetBatteryMV();
  //Particle.publish("Bateria", String( sensorReadings.millivolts));
}

// Convert sensorData to CSV String in US units
String WeatherSensors::sensorReadingsToCsvUS()
{
  String csvData =
//  String(Time.format(sensorReadings.unixTime, TIME_FORMAT_ISO8601_FULL))+
  String(sensorReadings.unixTime)+
  ","+
  String(sensorReadings.windDegrees)+
  ","+
  minimiseNumericString(String::format("%.1f",(float)sensorReadings.wind_metersph/1609.34),1)+
  ","+
  String(sensorReadings.humid)+
  ","+
  minimiseNumericString(String::format("%.1f",(((float)sensorReadings.airTempKx10-2731.5)*9.0/50.0+32.0)),1)+
  ","+
  minimiseNumericString(String::format("%.3f",((float)sensorReadings.rainmmx1000/25400)),3)+
  ","+
  minimiseNumericString(String::format("%.2f",(float)sensorReadings.barometerhPa/338.6389),2)+
  ","+
  minimiseNumericString(String::format("%.1f",(float)(sensorReadings.gust_metersph/1609.34)),1)+
  ","+
  minimiseNumericString(String::format("%.3f",(float)sensorReadings.millivolts/1000.0),3); // replace with voltage/lux
  //","+
  //String(sensorReadings.lux)
  //;
  return csvData;
}


// Convert sensorData to CSV String in US units
String WeatherSensors::sensorReadingsToCsvUS(sensorReadings_t readings)
{
 
  String csvData =
//  String(Time.format(readings.unixTime, TIME_FORMAT_ISO8601_FULL))+
  String(readings.unixTime) +
  ","+
  String(readings.windDegrees)+
  ","+
  minimiseNumericString(String::format("%.1f",(float)readings.wind_metersph/1609.34),1)+
  ","+
  String(readings.humid)+
  ","+
  minimiseNumericString(String::format("%.1f",(((float)readings.airTempKx10-2731.5)*9.0/50.0+32.0)),1)+
  ","+
  minimiseNumericString(String::format("%.3f",((float)readings.rainmmx1000/25400)),3)+
  ","+
  minimiseNumericString(String::format("%.2f",(float)readings.barometerhPa/338.6389),2)+
  ","+
  minimiseNumericString(String::format("%.1f",(float)(readings.gust_metersph/1609.34)),1)+
  ","+
  minimiseNumericString(String::format("%.3f",(float)sensorReadings.millivolts/1000.0),3);
  //","+
  //String(sensorReadings.lux);

  return csvData;
}

// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

//https://stackoverflow.com/questions/277772/avoid-trailing-zeroes-in-printf
String WeatherSensors::minimiseNumericString(String ss, int n) {
    int str_len = ss.length() + 1;
    char s[str_len];
    ss.toCharArray(s, str_len);

    //Serial.println(s);
    char *p;
    int count;

    p = strchr (s,'.');         // Find decimal point, if any.
    if (p != NULL) {
        count = n;              // Adjust for more or less decimals.
        while (count >= 0) {    // Maximum decimals allowed.
             count--;
             if (*p == '\0')    // If there's less than desired.
                 break;
             p++;               // Next character.
        }

        *p-- = '\0';            // Truncate string.
        while (*p == '0')       // Remove trailing zeros.
            *p-- = '\0';

        if (*p == '.') {        // If all decimals were zeros, remove ".".
            *p = '\0';
        }
    }
    return String(s);
}
