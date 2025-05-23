/*

ESP32S3 BME280 sensor NMEA2000 demo
skpang.co.uk 05/2025
 
BME280 sensor will output the following PGN on the NMEA2000 network:

PGN 130316,Temperature, Extended Range
PGN 130313 Humidity
PGN 130315 Pressure

For use with this board:
https://www.skpang.co.uk/products/esp32s3-can-bus-board


Ensure these libraries are installed first:
https://github.com/offspring/NMEA2000_esp32/tree/add-esp32s3
https://github.com/ttlappalainen/NMEA2000
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/master

*/

//#define CONFIG_IDF_TARGET_ESP32S3
#define ESP32_CAN_TX_PIN GPIO_NUM_10
#define ESP32_CAN_RX_PIN GPIO_NUM_11

#include "NMEA2000_esp32.h"
#include "NMEA2000_CAN.h"  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"
#include <Wire.h>
#include "SparkFunBME280.h"

#define ON  LOW
#define OFF HIGH

int LED_R = 39;
int LED_B = 40;
int LED_G = 38;

int I2C_SCL=18;   // I2C pin number
int I2C_SDA =8;

BME280 bme280;
unsigned long delayTime;
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={130316L,130313L,130315L,0};


// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler TemperatureScheduler(false,2000,500);
tN2kSyncScheduler PressureScheduler(false,2000,510);
tN2kSyncScheduler HumidityScheduler(false,2000,520);

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  TemperatureScheduler.UpdateNextTime();
  PressureScheduler.UpdateNextTime();
  HumidityScheduler.UpdateNextTime();
}

void setup() {
    
    // Set up serial for debugging
    Serial.begin(115200);

    pinMode(LED_R,OUTPUT);
    pinMode(LED_G,OUTPUT);
    pinMode(LED_B,OUTPUT);

    digitalWrite(LED_B, OFF);
    digitalWrite(LED_G, OFF);

    digitalWrite(LED_R, ON);
    delay(200); 
    digitalWrite(LED_R, OFF);
    
    digitalWrite(LED_G, ON);
    delay(200); 
    digitalWrite(LED_G, OFF);
    
    digitalWrite(LED_B, ON);
    delay(200); 
    digitalWrite(LED_B, OFF);
    
    Serial.println("ESP32 S3 NMEA2000 BME280 sensor. Temperature, humidity and pressure");
    Serial.println("skpang.co.uk 05/2025");

    Wire.begin(I2C_SDA, I2C_SCL);
    if (bme280.beginI2C() == false) //Begin communication over I2C
    {
      Serial.println("The BME280 sensor did not respond. Please check wiring.");
      while(1)
      {
            digitalWrite(LED_R, ON);
            delay(700); 
            digitalWrite(LED_R, OFF);
            delay(700); 
      }
  }
    // Set Product information
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Simple temp monitor",  // Manufacturer's Model ID
                                 "1.2.0.21 (2022-09-30)",  // Manufacturer's Software version code
                                 "1.1.0.0 (2022-09-30)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(112233, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75, // Device class=Sensor Communication Interface. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2040 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
  NMEA2000.SetForwardStream(&Serial);
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  //NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  NMEA2000.SetMode(tNMEA2000::N2km_SendOnly,22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  //NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
}    

void loop() {

  SendN2kTemperature();
  NMEA2000.ParseMessages();

}

// *****************************************************************************
double ReadTemp() {
  double tempC;
  double k;

  digitalWrite(LED_B, ON);
  tempC= bme280.readTempC();
  k =  CToKelvin(tempC);

  Serial.print("Temperature : ");
  Serial.print(tempC);
  Serial.print(" C  ");
  Serial.print(k);
  Serial.println(" K");

  delay(20);
  digitalWrite(LED_B, OFF);
  return k; 

}
// *****************************************************************************
double ReadHumidity() {
  double h;

  h = bme280.readFloatHumidity();
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println(" %");

  return h; 
}
// *****************************************************************************
double ReadPressure() {
  double p;

  p = bme280.readFloatAltitudeMeters();
  Serial.print("Pressure: ");
  Serial.print(p);
  Serial.println(" m");

  return p; 
}

// *****************************************************************************
void SendN2kTemperature() {
  tN2kMsg N2kMsg;

  if ( TemperatureScheduler.IsTime() ) {
    TemperatureScheduler.UpdateNextTime();
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_OutsideTemperature, ReadTemp(),0);
    NMEA2000.SendMsg(N2kMsg);
  }
  
  if ( HumidityScheduler.IsTime() ) {
     HumidityScheduler.UpdateNextTime();
     SetN2kPGN130313(N2kMsg, 1, 1, N2khs_OutsideHumidity, ReadHumidity(),0);
     NMEA2000.SendMsg(N2kMsg);
  }
  
  if ( PressureScheduler.IsTime() ) {
    PressureScheduler.UpdateNextTime();
    SetN2kSetPressure(N2kMsg,1, 1,N2kps_Atmospheric, ReadPressure());
    NMEA2000.SendMsg(N2kMsg);
  }
}
