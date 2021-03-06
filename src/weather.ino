float prom_o3=0;
#include "Particle.h"
#include <WeatherSensors.h>
#include "IoTNode.h"
#include "SdCardLogHandlerRK.h"
#include "Ubidots.h"
#include "PMS7003-Particle-Sensor-Serial.h"

PMS7003Serial<USARTSerial> pms7003(Serial1, D7);
//#ifndef TOKEN
//  #define TOKEN "BBFF-EYiaDYTpXbmJByMQqjUVXaLtMQ2w4S"
//#endif
unsigned long tiempo1=0;//timer para promediar los sensores spec
    unsigned long tiempo2=0;
    unsigned long tiempo3=0;
    float prom_o3P;
    float prom_o3_past=0;
    float prom_coP;
    float prom_co_past=0;
    float prom_no2P;
    float prom_no2_past=0;
    float prom_so2P;
    float prom_so2_past=0;
const char* WEBHOOK_NAME = "Ubidots";
Ubidots ubidots("webhook", UBI_PARTICLE);
//const char * webhook_name="Ubidots";
//Ubidots ubidots(TOKEN, UBI_PARTICLE);
#define SERIAL_DEBUG
int n=0;
#ifdef SERIAL_DEBUG// conversion debug_print --> serial_print
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

const int SD_CHIP_SELECT = N_D0;// memoria sd
SdFat sd;//La biblioteca Arduino SdFat proporciona acceso de lectura / escritura a sistemas de archivos FAT16 / FAT32 en tarjetas flash SD / SDHC.
SdCardPrintHandler printToSd(sd, SD_CHIP_SELECT, SPI_FULL_SPEED);//sd es el SdFatobjeto, como se describe en la sección anterior
//SD_CHIP_SELECTes el pin conectado al pin CS / SS. En el código anterior, SD_CHIP_SELECTse establece en A2.
//SPI_FULL_SPEEDdetermina la velocidad a utilizar, ya sea completa o SPI_HALF_SPEED.

STARTUP(printToSd.withMaxFilesToKeep(3000));//controla el tamaño de archivos

#define SENSOR_SEND_TIME_MS 10000// define un tiempo en milisegundos de envio de datos de los sensores
#define SENSOR_POLL_TIME_MS 2000// define un tiempo en ms de tiempo de sondeo de sensor

#define IOTDEBUG

LEDStatus fadeRed(RGB_COLOR_RED, LED_PATTERN_FADE, LED_SPEED_NORMAL, LED_PRIORITY_IMPORTANT);//led paradeante solor rojo

const int firmwareVersion = 0;

SYSTEM_THREAD(ENABLED);//System Thread es una configuración del sistema que garantiza que el bucle de la aplicación no sea interrumpido por el procesamiento en segundo plano del sistema y la administración de la red

// Using SEMI_AUTOMATIC mode to get the lowest possible data usage by
// registering functions and variables BEFORE connecting to the cloud.
//SYSTEM_MODE(SEMI_AUTOMATIC);

// Use structs defined in WeatherSensors.h
sensorReadings_t sensorReadings;//objeto de la libreria de sensores
config_t config;//objeto de la libreria de sensores

//********CHANGE BELOW AS NEEDED**************
// Change this value to force hard reset and clearing of FRAM when Flashing
// You have to change this value (if you have flashed before) for the TS channel to change
const int firstRunTest = 1122124;
//********CHANGE ABOVE AS NEEDED**************


// This is the index for the updateTSChan
int returnIndex;

byte messageSize = 1;

Timer pollSensorTimer(SENSOR_POLL_TIME_MS, capturePollSensors);//declaramos el timer del sondeo de los sensores y lo capturamos

Timer sensorSendTimer(SENSOR_SEND_TIME_MS, getResetAndSendSensors);//declaramos el timer de envio de envio de timer de los sensores enviamos el tiempo de envio y su modificacion

Timer unpluggedTimer(5000,unplugged);

WeatherSensors sensors; //Interrupts for anemometer and rain bucket
// are set up here too

IoTNode node;//objeto nodo de libreria iotnode

// // Create FRAM array and ring
framArray framConfig = node.makeFramArray(1, sizeof(config));

framRing dataRing = node.makeFramRing(300, sizeof(sensorReadings));

bool readyToGetResetAndSendSensors = false;//declaramos el listo para el mandar y resetear de los sensores y su captura
bool readyToCapturePollSensors = false;
bool tickleWD = false;

unsigned long timeToNextSendMS;

String deviceStatus;
String i2cDevices;
bool resetDevice = false;

String i2cNames[] =// arreglo con los nombres de dispositivos i2c
{
    "RTC",
    "Exp",
    "RTC EEPROM",
    "ADC",
    "FRAM",
    "AM2315",
    "MPL3115",
    "TSL2591",
    "ADS1115"
};

byte i2cAddr[]=//direcciones i2c de los sensores
{
    0x6F, //111
    0x20, //32
    0x57, //87
    0x4D, //77
    0x50, //80
    0x5C, //
    0x60,
    0x29,
    0x48
    
};

/* number of elements in `array` */
static const size_t i2cLength = sizeof(i2cAddr) / sizeof(i2cAddr[0]);// declaramos una constante estatica del tamaño de arreglo de direcciones

bool i2cExists[]=
{
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  false,
  false
};

// check i2c devices with i2c names at i2c address of length i2c length returned in i2cExists
bool checkI2CDevices()
{
  byte error, address;//constante error y address
  bool result = true;// variable result
  for (size_t i; i<i2cLength; ++i)// recorremos el arreglo de las direcciones
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    address = i2cAddr[i];// address es igaual a los datos guardados en nuestro arreglo i2cAdd

    Wire.beginTransmission(address);//Inicie una transmisión al dispositivo esclavo I 2 C con la dirección dada
    error = Wire.endTransmission();// igualamos a error cuando esa transmision se termina

    //Try again if !error=0
    if (!error==0)
    {
      delay(10);
      Wire.beginTransmission(address);//Inicie una transmisión al dispositivo esclavo I 2 C con la dirección dada
      error = Wire.endTransmission();// igualamos a error cuando esa transmision se termina
    }

    //Try reset if !error=0
    if (!error==0)//intentamos de nuevo si el error no es 0
    {
      Wire.reset();// reseteamos la transmiision
      Wire.beginTransmission(address);//volvemos a iniciar la transmision 
      error = Wire.endTransmission();//igualamos a error cuando esa transmision se termina
    }
 
    if (error == 0)
    {
      DEBUG_PRINTLN(String("Device "+i2cNames[i]+ " at"+" address:0x"+String(address, HEX)));//imprimimos el dispositivo, nombre, direccion
      i2cExists[i]=true;// cambiamos de false a true en el arreglo
    }
    else
    {
      DEBUG_PRINTLN(String("Device "+i2cNames[i]+ " NOT at"+" address:0x"+String(address, HEX)));//se imprime que no esta en la direccion
      i2cExists[i]=false;//dejamos el false en el arreglo
      result = false;
    }
  }
  return result;
}


void printI2C(int inx)//imprimir i2c
{
    for (int i=0; i<i2cLength; i++)
        {
          if (i2cAddr[i] == inx)//comp lo que tenemos en la posicion i a inx
          {
              DEBUG_PRINTLN(String("Device "+i2cNames[i]+ " at"+" address:0x"+String(i2cAddr[i], HEX)));//imprimimos el dispositivo con su direccion
          }
        }        
}

void scanI2C()
{
  byte error, address;//constantes
  int nDevices;//num de dispositivos
 
  DEBUG_PRINTLN("Scanning...");//imprimios
  nDevices = 0;//num de dispositivos =0
  for(address = 1; address < 127; address++ )// ciclo for donde add es igual a 1 add tiene q ser menor a 127 y add++
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)//si el error es igual a 0
    {
      printI2C(address);//le damos la direccion a printi2c q despues comprobara si es igal a la q esta en el arreglos de i2cAddr
 
      nDevices++;//sumamos  un dispositivo mas
    }
    else if (error==4)//si el error es 4
    {
      DEBUG_PRINT("Unknown error at address 0x");//imprimimos 
      if (address<16)//si es menor que 16
        DEBUG_PRINT("0");//imprimimos
      DEBUG_PRINTLN(address,HEX);//su direccion en hexadecimal
    }    
  }
  if (nDevices == 0)//si no hay dispositivos
    DEBUG_PRINTLN("No I2C devices found\n");//imprimimos
  else
    DEBUG_PRINTLN("done\n");
}


void connect()//Agregar una rutina de conexión explícita que debe funcionar antes de que se ejecute el resto del código
{
  #if Wiring_Cellular
  bool cellready=Cellular.ready();
  if (!cellready)
  {
    DEBUG_PRINTLN("Attempting to connect cellular...");
    Cellular.on();
    Cellular.connect();
    waitFor(Cellular.ready,180000);
    if (!Cellular.ready())
    {
    DEBUG_PRINTLN("Cellular not ready - resetting");
    delay(200);
    System.reset();
    }
  }
  else
  {
    DEBUG_PRINTLN("Cellular ready");
  }
  #endif
  
  #if Wiring_WiFi
  if (!WiFi.hasCredentials())//si no tenemos las credenciales
  {
    DEBUG_PRINTLN("Please add WiFi credentials");
    DEBUG_PRINTLN("Resetting in 60 seconds");
    delay(60000);
    System.reset();
  }
  bool wifiready=WiFi.ready();
  if (!wifiready)
  {
    DEBUG_PRINTLN("Attempting to connect to WiFi...");
    WiFi.on();
    WiFi.connect();
    waitFor(WiFi.ready,60000);
    if (!WiFi.ready())
    {
    DEBUG_PRINTLN("WiFi not ready - resetting");
    delay(200);
    System.reset();
    }
  }
  else
  {
    DEBUG_PRINTLN("Cellular ready");
  }
  #endif

  bool partconnected=Particle.connected();
  if (!partconnected)
  {
    DEBUG_PRINTLN("Attempting to connect to Particle...");
    Particle.connect();
    // Note: that conditions must be a function that takes a void argument function(void) with the () removed,
    // e.g. Particle.connected instead of Particle.connected().
    waitFor(Particle.connected,60000);
    if (!Particle.connected())
    {
      DEBUG_PRINTLN("Particle not connected - resetting");
      delay(200);
      System.reset();
    } 
  }
  else
  {
    DEBUG_PRINTLN("Particle connected");
  }
}
Adafruit_ADS1115 ads;//ads1115 con direccion default
// setup() runs once, when the device is first turned on.
void setup() {
  ubidots.setDebug(true);
  // register cloudy things
  Particle.variable("version",firmwareVersion);
  Particle.variable("devicestatus",deviceStatus);

  Serial.begin();//inicializamos el puerto serial
  //Serial1.begin(115200);
   
  ads.begin();
ads.setGain(GAIN_TWOTHIRDS); 
  node.begin();
  node.setPowerON(EXT3V3,true);
  node.setPowerON(EXT5V,true);
  tiempo2= millis();
  #ifdef IOTDEBUG
  delay(5000);
 
  checkI2CDevices();//le decimos q revise los dispositivos i2c 
  scanI2C();//y haga el escaneo
  #endif

    // Check for I2C devices again
  if (!node.ok())//si en nodo no esta bien 
  {
    #ifdef IOTDEBUG
    // Particle.publish("Unplugged","Plug the device into the IoT Node",PRIVATE);
    DEBUG_PRINTLN("Plug the device into the IoT Node");//conecta el dispositivo en el nodo
    #endif
    deviceStatus="Device is not plugged into the IoTNode";//el device estatus es igual a los dispositivos no estan conectados
    fadeRed.setActive(true);//encendemos el led parpadeante de color rojo
    DEBUG_PRINTLN("Resetting in 10 seconds");//imprimimos
    delay(10000);
    System.reset();//reseteamos el sistema
  }
  else
  {
    
    connect();
    framConfig.read(0, (uint8_t*)&config);

      // Code for initialization of the IoT Node
      // i.e. the first time the software runs
      // 1. A new ThingSpeak channel is created
      // 2. The channel id and keys are Saved
      // 3. a firstRunTest variable is saved in persistent memory as a flag to indicate
      // that the IoT node has been set up already.

    if (config.testCheck != firstRunTest)
    {
        config.testCheck = firstRunTest;
        /// Defaults
        config.particleTimeout = 20000;
        // Save to FRAM
        framConfig.write(0, (uint8_t*)&config);     
    }
      // end of first run code.

      if (syncRTC())//si el reloj ha sido sincronizado
      {
        DEBUG_PRINTLN("RTC sync'ed with cloud");//imprime q el reloj esta sinc con la nube
      }
      else
      {
        DEBUG_PRINTLN("RTC not sync'ed with cloud");//imprime q el reloj no esta sinc con la nube
      }
      // load pointers
      dataRing.initialize();
      sensors.begin();
      pollSensorTimer.start();;//comienza a sondear
      sensorSendTimer.start();  //comienza a mandar datos
  }
}
int cont_vgas=0;
int cont=0;
double sumvgasco, sumvgaso3, sumvgasno2, sumvgasso2;
double promvgaso3, promvgasno2, promvgasco, promvgasso2;
  double sum_1=0;
  double sum_2=0;
  double sum_3=0;
  double sum_4=0;
  double prom_conc_o3=0.0;
  double prom_conc_co=0;
  double prom_conc_no2=0;
  double prom_conc_so2=0;
// Note that CSV format is:
// unixTime,windDegrees,wind_speed,humidity,air_temp,rain,pressure,wind_gust,millivolts,lux
unsigned long last = 0;
unsigned long last_pm_reading = 0;
unsigned long last_25 = 0;
unsigned long last_pm_reading_25 = 0;
unsigned long last_10 = 0;
unsigned long last_pm_reading_10 = 0;

void loop() {
  
   double multiplier = 0.1875F; //milli Volts per bit for ADS1115
  //double multiplier = 3.0F; //milli Volts per bit for ADS1105

  double adc0, adc1, adc2, adc3;
  double av0, av1, av2, av3;
 
  
  
 adc0 = ads.readADC_SingleEnded(0);
  av0 = (adc0 * multiplier)/1000;
  adc1 = ads.readADC_SingleEnded(1);
  av1= (adc1 * multiplier)/1000;
  adc2 = ads.readADC_SingleEnded(2);
  av2= (adc2 * multiplier)/1000;
  adc3 = ads.readADC_SingleEnded(3);
  av3= (adc3 * multiplier)/1000;
  

  if (readyToGetResetAndSendSensors)//si esta listo para reset y mandar datos
  {
    sensors.getAndResetAllSensors();//reseteamos los sensores y los mandamos llamar
    char msg[512]; //cadena de 256 bytes
    uint32_t UT=sensorReadings.unixTime;//lectura del unixtime
    float VV= sensorReadings.wind_metersph * 0.001;//lectura de la velocidad del viento
    float Precip = sensorReadings.rainmmx1000 / 1000.0; //lectura de precipitacion
    float DV= sensorReadings.windDegrees; //lectura de direccion del viento
    float Temp = (sensorReadings.airTempKx10 / 10.0)-273.15; //lectura de temperatura
    uint16_t mVB=sensorReadings.millivolts;//lectura de voltaje
    uint16_t Hum=sensorReadings.humid;//lectura de humedad
    unsigned int PM1_0 = pm1();
    unsigned int PM2_5 =pm2_5();
    unsigned int PM10 = pm10();
    //uint16_t ozo=sensorReadings.ozone;
     
adc0 = ads.readADC_SingleEnded(0);
  av0 = (adc0 * multiplier)/1000;
  adc1 = ads.readADC_SingleEnded(1);
  av1= (adc1 * multiplier)/1000;
  adc2 = ads.readADC_SingleEnded(2);
  av2= (adc2 * multiplier)/1000;
  adc3 = ads.readADC_SingleEnded(3);
  av3= (adc3 * multiplier)/1000;
double concentration_o3 =  -59.714 * (av0 - 	1.590161);
cont++;
sum_1 += concentration_o3;
prom_conc_o3= sum_1/cont;
if (prom_conc_o3 >= 0){
prom_o3P = prom_conc_o3;
prom_o3_past= prom_conc_o3;
}else
{
  prom_o3P= prom_o3_past;
}
double concentration_co = 4149.377593 * (av1 - 1.631584333333333);
sum_2 += concentration_co;
prom_conc_co= sum_2/cont;
if (prom_conc_co >= 0){
prom_coP = prom_conc_co;
prom_co_past= prom_conc_co;
}else
{
  prom_coP= prom_co_past;
}
double concentration_no2 =  -95.474417152551887958861983137308 * (av2 - 1.624044);
sum_3+= concentration_no2;
prom_conc_no2= sum_3/cont;
if (prom_conc_no2 >= 0){
prom_no2P = prom_conc_no2;
prom_no2_past= prom_conc_no2;
}else
{
  prom_no2P= prom_no2_past;
}
double concentration_so2 =  346.26038781163434903047091412742 * (av3 - 1.6933555555555);
sum_4+= concentration_so2;
prom_conc_so2= sum_4/cont;
if (prom_conc_so2 >= 0){
prom_so2P = prom_conc_so2;
prom_so2_past= prom_conc_so2;
}else
{
  prom_so2P= prom_so2_past;
}
    
    
snprintf(msg, sizeof(msg) , //imprimimos la cadena 
"{\"UT\": %u, \"VV\": %.1f , \"Precip\": %.1f , \"DV\": %.1f , \"Temp\": %.1f ,\"Hum\": %u  ,\"mVB\": %u , \"O3\": %.6f ,\"CO\": %.6f ,\"NO2\": %.6f ,\"SO2\": %.6f ,\"VGASO3\": %.14f ,\"VGASCO\": %.14f ,\"VGASNO2\": %.14f ,\"VGASSO2\": %.14f ,\"PM1\": %u ,\"PM2.5\": %u ,\"PM10\": %u ,}" , 
UT,
VV, 
Precip, 
DV ,
Temp, 
Hum,  
mVB,
prom_o3P,
prom_coP,
prom_no2P,
prom_so2P,
av0,
av1,
av2,
av3,
PM1_0,
PM2_5,
PM10

);

 Particle.publish("sensors", msg, PRIVATE);//mandamos los datos a la nube de particle
 ubidots.add("UnixTime", UT);//mandamos lo datos a la nube de ubidots
 ubidots.add("Velocidad del Viento", VV); 
 ubidots.add("Precipitacion", Precip);
 ubidots.add("Direccion del viento", DV);
 ubidots.add("Temperatura", Temp);
 ubidots.add("Humedad", Hum);
 ubidots.add("Milivolts", mVB);
 //ubidots.send(webhook_name, PUBLIC); 
ubidots.send(WEBHOOK_NAME, PUBLIC); 
    

    String currentCsvData = sensors.sensorReadingsToCsvUS();

    // Consider putting the SD logging in the IoTNode library
    printToSd.println(currentCsvData);

    DEBUG_PRINTLN(currentCsvData);

    readyToGetResetAndSendSensors = false;

    if (tickleWD)
    {
      node.tickleWatchdog();
      tickleWD = false;
    }

    readyToGetResetAndSendSensors = false;
    #ifdef IOTDEBUG
    DEBUG_PRINTLN("readyToGetResetAndSendSensors");
    #endif
    // Update status information
    deviceStatus = 
    String(config.testCheck)+"|"+

    String(config.unitType)+"|"+
    String(config.firmwareVersion)+"|"+
    String(config.particleTimeout)+"|"+
    String(config.latitude)+"|"+
    String(config.longitude)+"|"+
    i2cDevices;

  }

  if (readyToCapturePollSensors)//si esta listo para sondear los sensores
  {
    sensors.captureTempHumidityPressure();//capturamos la temperatura, humedad, presion
    sensors.captureWindVane();// capturamos la veleta del aire
    //sensors.captureLightLux();//
    sensors.captureBatteryVoltage();//capturamos voltage de la bateria
    readyToCapturePollSensors = false;
    #ifdef IOTDEBUG
    Particle.publish("Capturing sensors",PRIVATE);
    DEBUG_PRINTLN("capture");
    #endif
  }
  // If flag set then reset here
  if (resetDevice)
  {
    System.reset();
  }

}

void capturePollSensors()// funcion de la libreria para sondar sensores
{
  // Set the flag to poll the sensors
  readyToCapturePollSensors = true;
}

void getResetAndSendSensors()//funcion de la libreria para restear y mandar sensores
{
  // Set the flag to read and send data.
  // Has to be done out of this Timer thread
  timeToNextSendMS = millis();
  readyToGetResetAndSendSensors = true;
}

bool syncRTC()// sincroniza el reloj en tiempo real
{
    uint32_t syncNow;// variable 32 bits 
    bool sync = false;// variable sync igualada a false
    unsigned long syncTimer = millis();

    do
    {
      Particle.process();//comprueba el módulo Wi-Fi en busca de mensajes entrantes de la nube y procesa cualquier mensaje que haya entrado.
      delay(100);
    } while (Time.now() < 1465823822 && millis()-syncTimer<500);

    if (Time.now() > 1465823822)
    {
        syncNow = Time.now();//pone el tiempo en la memoria
        node.setUnixTime(syncNow);//sincroniza el tiempo del node
        sync = true;// la sincronizacion fue true
    }

    if (!sync)
    {
        #ifdef DEBUG
        Particle.publish("Time NOT synced",String(Time.format(syncNow, TIME_FORMAT_ISO8601_FULL)+"  "+Time.format(node.unixTime(), TIME_FORMAT_ISO8601_FULL)),PRIVATE);//publicamosa evento de tiempo no sinc
        #endif
    }
    return sync;
}

void unplugged()
{
  #ifdef IOTDEBUG
  Particle.publish("Unplugged","Plug the device into the IoT Node",PRIVATE);//publica un eveento privado
  DEBUG_PRINTLN("Plug the device into the IoT Node");//imprime el mensaje
  #endif

}
// float spec_O3(){
//   int cont_o3=0;
//     float vref_o3 = 0;
//     float vgas_o3 = 0;
//     float temp_o3 = 0;
//     float tempVin = 0;
//     float Vvref_o3 =0;
//     float Vvgas_o3 =0;
//     float Vtemp_o3=0;
    
//     float sum_o3 =0;
    
// Vvgas_o3 = (ads.readADC_SingleEnded(0)*0.1875)/1000.0;
    
//     float concentration_o3 = -59.714184029560909661993832719073 * (Vvgas_o3 - 1.6546);//PPM
 
//     cont_o3++;
//     sum_o3 += concentration_o3;
//     prom_o3 =sum_o3 / cont_o3;
    
//     return Vvgas_o3;
   
// }



// float Specs(){
//   double multiplier = 0.1875F; //milli Volts per bit for ADS1115
//   //double multiplier = 3.0F; //milli Volts per bit for ADS1105
// char pm[256];
//   short adc0, adc1, adc2, adc3;
//   double av0, av1, av2, av3;
  
//   adc0 = ads.readADC_SingleEnded(0);
//   av0 = adc0 * multiplier;
//   adc1 = ads.readADC_SingleEnded(1);
//   av1= adc1 * multiplier;
//   adc2 = ads.readADC_SingleEnded(2);
//   av2= adc2 * multiplier;
//   adc3 = ads.readADC_SingleEnded(3);
//   av3= adc3 * multiplier;

//   snprintf(pm, sizeof(pm) , 
// 			"{\"A0:\": %.15f, \"A1\": %.15f, \"A2\": %.15f , \"A3\": %.15f }" , 

// av0,
// av1,
// av2,
// av3
// );

// // Particle.publish("sensors", msg, PRIVATE);	
// //   Particle.publish("VGAS A0",String(av0,7)); 
// //   Particle.publish("VGAS A1",String(av1,7)); 
// //   Particle.publish("VGAS A2",String(av2,7)); 
// //   Particle.publish("VGAS A3",String(av3,7)); 
  
//  // Serial.println(" ");
// }
unsigned int pm1(){
  unsigned long now = millis();
  if (pms7003.Read()) {
    last_pm_reading = now;
  }

  if ((now - last) > 5000) {
    // Let us be generous. Active state the device
    // reports at least every 2.3 seconds.
    if ((now - last_pm_reading) > 5000) {
      //Serial.println("No reading for at least 10 seconds!");
    } else {
        unsigned int pm1 = pms7003.GetData(pms7003.pm1_0);
        return pm1;		

    }
    last = now;
  }
}
unsigned int pm2_5(){
  unsigned long now_25 = millis();
  if (pms7003.Read()) {
    last_pm_reading_25 = now_25;
  }

  if ((now_25 - last_25) > 5000) {
    // Let us be generous. Active state the device
    // reports at least every 2.3 seconds.
    if ((now_25 - last_pm_reading_25) > 5000) {
      //Serial.println("No reading for at least 10 seconds!");
    } else {
        unsigned int pm2_5 = pms7003.GetData(pms7003.pm2_5);
        return pm2_5;		

    }
    last = now_25;
  }
}
unsigned int pm10(){
  unsigned long now_10 = millis();
  if (pms7003.Read()) {
    last_pm_reading_10 = now_10;
  }

  if ((now_10 - last_10) > 5000) {
    // Let us be generous. Active state the device
    // reports at least every 2.3 seconds.
    if ((now_10 - last_pm_reading_10) > 5000) {
      //Serial.println("No reading for at least 10 seconds!");
    } else {
        unsigned int pm10 = pms7003.GetData(pms7003.pm10);
        return pm10;		

    }
    last = now_10;
  }
}
