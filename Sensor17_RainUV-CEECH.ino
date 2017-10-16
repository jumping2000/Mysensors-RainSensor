 /**
 *     PROJECT: MySensors / Small battery sensor low power 8 mhz
 *     PROGRAMMER: Jumping
 *     DATE: october 17, 2016/ last update: october 16, 2017
 *     FILE: Sensor17_RainUV-CEECH.ino
 *     LICENSE: Public domain
 *    
 *     Hardware: ATMega328p board w/ NRF24l01
 *        and MySensors 2.1
 *            
 *    Special:
 *        program with Arduino Pro 3.3V 8Mhz!!!
 *        
 *    Summary:
 *        low power (battery)
 *        BH1750FVI Light sensor
 *        ML8511
 *        voltage meter for battery
 *    
 *    Remarks:
 *
 *  Connect the following MP8511 breakout board to Arduino:
 *   3.3V = 3.3V
 *   OUT = A3
 *   GND = GND
 *   EN = 3.3V
 *   3.3V = A1
 *   These last two connections are a little different. Connect the EN pin on the breakout to 3.3V on the breakout.
 *   This will enable the output. Also connect the 3.3V pin of the breakout to Arduino pin 1
 *
*******************************
 *
 * REVISION HISTORY
 * Version 1.0 - idefix
 * 
 * DESCRIPTION
 * Arduino BH1750FVI Light sensor
 * communicate using I2C Protocol
 * this library enable 2 slave device addresses
 * Main address  0x23
 * secondary address 0x5C
 * connect the sensor as follows :
 *
 *   VCC  >>> 5V
 *   Gnd  >>> Gnd
 *   ADDR >>> NC or GND  
 *   SCL  >>> A5
 *   SDA  >>> A4
 * http://www.mysensors.org/build/light
 * 
 * USING CEECH BOARD
 */
// Enable debug prints to serial monitor
#define MY_DEBUG 
// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69
// change NRF24 pin
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 8
#define MY_RF24_CHANNEL  76
// NODE ID
#define MY_NODE_ID 17
#define NODE_TXT "RAIN_17"  // Text to add to sensor name
//
#include <MySensors.h>  
#include <BH1750.h>
#include <Wire.h>
//#include <Time.h>
#include <TimeLib.h>

//-------------------------------------------------------------------
#define LTC4079_CHRG_PIN            A7      //analog input A7 on ATmega 328 is /CHRG signal from LTC4079
#define batteryVoltage_PIN          A0      //analog input A0 on ATmega328 is battery voltage
#define solarVoltage_PIN            A2      //analog input A2 is solar cell voltage
#define chargeCurrent_PIN           A6      //analog input A6 is battery charge current

#define uvPin                       A3      //Output from the sensor
#define refPin                      A1      //3.3V power on the Arduino board
#define tipSensor_PIN               3       // The reed switch you attached.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT tipSensor_PIN-2           // Usually the interrupt = pin -2 (on uno/nano anyway)


//-------------------------------------------------------------------
// Reference values for ADC and Battery measurements
const float VccMin          = 1.0*2.8 ;             // Minimum expected Vcc level, in Volts.
const float VccMax          = 1.0*4.2 ;             // Maximum expected Vcc level, in Volts. 
const float VccCorrection   = 3.32/3.3 ;            // Measured Vcc by multimeter divided by reported Vcc
float VccReference = 3.27 ;                         // voltage reference for measurement, definitive init in setup
//-------------------------------------------------------------------
#define LIGHT_CHILD_ID      1
#define UV_CHILD_ID         2
#define RAIN_CHILD_ID       3  // Ke
eps track of accumulated rainfall
#define BATT_CHILD_ID       7
//-------------------------------------------------------------------
  const float Threshold = 0.1 ;                   // send only if change > treshold
  const int heartbeat = 3 ;                       // heartbeat every hour (x times SLEEP_TIME)
  unsigned long lastHeartbeat = 0 ;
  const unsigned long SLEEP_TIME = 19*60*1000UL;   // Sleep time between reads (in milliseconds) - 20 minuti
//-------------------------------------------------------------------
  float     lastUV = -1;
  uint16_t  lastLux = -1;
  float     lastBattVoltage = -1;
  float     lastBattCurrent = -1;
  int       lastBattPct = 0;
  // flags to indicate if transmission is needed, heartbeat and/or changes > treshold
  boolean txLux = true ;
  boolean txUV = true ;
  boolean txRain = true ;  
  boolean txBattVoltage = true ;
  boolean txBattCurrent = true ;
//-------------------------------------------------------------------
  bool timeReceived = false;
  //unsigned long lastRequest=0, lastHour=0, lastMinute=0;
  float hwRainVolume = 0;                         // Current rainvolume calculated in hardware.
  int hwPulseCounter = 0;                         // Pulsecount recieved from GW
  float fullCounter = 0;                           // Counts when to send counter
  float bucketSize = 0.3;                           // Bucketsize mm, needs to be 1, 0.5, 0.25, 0.2 or 0.1
  boolean pcReceived = false;                     // If we have recieved the pulscount from GW or not 
  boolean reedState;                              // Current state the reedswitch is in
  boolean oldReedState;                           // Old state (last state) of the reedswitch
  unsigned long lastSend =0;                      // Time we last tried to fetch counter.
  //-------------------------------------------------------------------
  BH1750 lightSensor;                                         // lux meter
  //-------------------------------------------------------------------
  MyMessage luxMsg(LIGHT_CHILD_ID, V_LEVEL);                  // BH 1750 Light sensor (lux)
  MyMessage uvMsg(UV_CHILD_ID, V_UV);                         // ML8511 UV Sensor
  MyMessage rainMsg(RAIN_CHILD_ID, V_RAIN);                   // Bucket sensor
  MyMessage lastCounterMsg(RAIN_CHILD_ID,V_VAR1);
  //--------------------------------------------------------------------------------
  MyMessage batteryVoltageMsg(BATT_CHILD_ID, V_VOLTAGE);      // Battery voltage (V)
  MyMessage batteryCurrentMsg(BATT_CHILD_ID, V_CURRENT);      // Battery current (mA)
  //--------------------------------------------------------------------------------
void setup() { 
  Serial.begin(9600);
  Wire.begin();             // START I2C
  lightSensor.begin();      // LIGHT SENSOR
  pinMode(uvPin, INPUT);    // UV SENSOR
  pinMode(refPin, INPUT);
  pinMode(tipSensor_PIN, INPUT); //HW debouncing
  //-------------------------
  //attachInterrupt (digitalPinToInterrupt(tipSensor_PIN), sensorTipped, CHANGE);  // depending on location of the hall effect sensor may need CHANGE

  reedState = digitalRead(tipSensor_PIN); // Read what state the reedswitch is in
  oldReedState = reedState; // Set startup position for reedswitch 
  Serial.println("Startup completed");
}
//-------------------------------------------------------------------
void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("JMP "NODE_TXT, "1.0");
  wait(300);
  // Register all sensors to gateway (they will be created as child devices)
  present(BATT_CHILD_ID, S_POWER, "Batt " NODE_TXT);        // Battery parameters
  wait(300);
  present(LIGHT_CHILD_ID, S_LIGHT_LEVEL,"Light " NODE_TXT); // Light sensor
  wait(300);
  present(UV_CHILD_ID, S_UV, "UV " NODE_TXT);               // UV
  wait(300);
  present(RAIN_CHILD_ID, S_RAIN, "Rain " NODE_TXT);         // RAIN Sensor
}
//-------------------------------------------------------------------
void loop() {
  unsigned long currentTime = millis();
  //See if we have the counter/pulse from Domoticz - and ask for it if we dont.
  if (!pcReceived && (currentTime - lastSend > 5000)) {      
    request(RAIN_CHILD_ID, V_VAR1);
    lastSend=currentTime;
    return;
    }
  if (!pcReceived) {
    return;
    }   
  //Read if the bucket tipped over
  reedState = digitalRead(tipSensor_PIN);
  boolean tipped = oldReedState != reedState;
  //BUCKET TIPS!
  if (tipped==true) {
    Serial.println("The bucket has tipped over...");
    oldReedState = reedState;
    hwRainVolume = hwRainVolume + bucketSize;
    send(rainMsg.set((float)hwRainVolume,1));
    wait(1000);
    fullCounter = fullCounter + bucketSize;
    //Count so we send the counter for every 1mm
    if(fullCounter >= 1){
      hwPulseCounter++;
      send(lastCounterMsg.set(hwPulseCounter));
      wait(1000);
      fullCounter = 0;
      }
  }
  if (tipped==false) {
    readLux();
    readUV();
    //readRain();
    readVoltages();
    sendSensors();
    }
  //-------------------------
  lastSend=currentTime;
  sleep(INTERRUPT, CHANGE, SLEEP_TIME); //The interupt can be CHANGE or FALLING depending on how you wired the hardware.
}
//------------------------------------------------------------------
//Read if we have a incoming message.
void receive(const MyMessage &msg) {
    if (msg.type==V_VAR1) {
    hwPulseCounter = msg.getULong();
    hwRainVolume = hwPulseCounter;
    pcReceived = true;
    #ifdef MY_DEBUG
      Serial.print("Received last pulse count from gw: ");
      Serial.println(hwPulseCounter);
    #endif       
    }
}
//-------------------------------------------------------------------
void readLux(void)
{
   uint16_t lux = lightSensor.readLightLevel();             //Get Lux value
   if (abs(lux  - lastLux) >= Threshold) {
      lastLux = lux;
      txLux= true;
   }   
   #ifdef MY_DEBUG
      Serial.print("BH1750 lux: ");
      Serial.println(lux);
   #endif
}

//-------------------------------------------------------------------
void readUV(void)
{
  int uvLevel = averageAnalogRead(uvPin);
  int refLevel = averageAnalogRead(refPin);
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.97, 2.9, 0.0, 15.0); //Convert the voltage to a UV intensity level
  if (abs(uvIntensity  - lastUV) >= Threshold) {
      lastUV = uvIntensity;
      txUV=true;
  }  
  #ifdef MY_DEBUG
    Serial.print("UV Intensity (mW/cm^2): ");
    Serial.print(uvIntensity);
    Serial.println();
  #endif
}
//-------------------------------------------------------------------
void readVoltages(void)
{ 
   float voltage = readVcc(); // actual VOLTAGE VCC
   #ifdef MY_DEBUG
     Serial.print("Vcc ");
     Serial.print(voltage);
     Serial.println("V ");
   #endif
   //------------------
   float batteryChargeCurrent = ((float)analogRead(chargeCurrent_PIN) * voltage/1024)/ 3.3 * 250; // CURRENT battery
   if (abs(batteryChargeCurrent  - lastBattCurrent) >= Threshold) {
      lastBattCurrent = batteryChargeCurrent;
      txBattCurrent= true;
   }   
   #ifdef MY_DEBUG
     Serial.print("Charge current ");
     Serial.print(batteryChargeCurrent);
     Serial.println("mA ");
   #endif
   //------------------
   float batteryVoltage = ((float)analogRead(batteryVoltage_PIN) * voltage/1024)* 2; // VOLTAGE battery
   if (abs(batteryVoltage  - lastBattVoltage) >= Threshold) {
      lastBattVoltage = batteryVoltage;
      txBattVoltage= true;
   } 
   #ifdef MY_DEBUG
     Serial.print("Battery voltage ");
     Serial.print(batteryVoltage);
     Serial.println("V ");
   #endif
   //------------------
   int charge = (int)analogRead(LTC4079_CHRG_PIN);
   #ifdef MY_DEBUG
     Serial.print("CHRG ");
     Serial.println(charge);
   #endif
}
//-------------------------------------------------------------------
void sendSensors(void)
{
    lastHeartbeat++ ;                                       // update Heartbeatcount every call
    if ( lastHeartbeat > heartbeat) {                       // if heartbeat update all sensors & battery status
        txBattVoltage = txBattCurrent = txUV = txLux = txRain = true ;
        int batteryPcnt = 100.0*(lastBattVoltage - VccMin)/(VccMax - VccMin); // Convert voltage to percentage
        //int batteryPcnt = static_cast<int>(((lastVoltage-VccMin)/(VccMax - VccMin))*100.);
        sendBatteryLevel(batteryPcnt, true);
        lastHeartbeat = 0;
        wait(200);
        }
    if (txUV){
        send(uvMsg.set(lastUV, 2));                             // Send UV index
        txUV = false;
        wait(200);
        }
     if (txLux){
        send(luxMsg.set(lastLux));                              // Send lux
        txLux = false ;
        wait(200);
        }
    if (txBattVoltage){
        send(batteryVoltageMsg.set(lastBattVoltage, 2));        // Send battery V
        txBattVoltage = false;
        wait(200);
        }
    if (txBattCurrent){
        send(batteryCurrentMsg.set(lastBattCurrent, 2));        // Send battery I
        txBattCurrent = false;
        wait(200);
        }
    if (txRain) {                                               // Send Rain
        send(rainMsg.set((float)hwRainVolume,1));
        wait(800);
        send(lastCounterMsg.set(hwPulseCounter));
        txRain = false;
        wait(200);
        }   
}
//-------------------------------------------------------------------
float readVcc() 
{
  signed long resultVcc;
  float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10);                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                 // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1126400L / resultVcc;    // Back-calculate AVcc in mV
  resultVccFloat = (float) resultVcc / 1000.0; // Convert to Float

  return resultVccFloat;
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

/* Ceech board specifics for reference:
voltmeters on both battery and solar cell connections
They are connected to analog inputs A0 and A2 on the ATmega328p. The voltage dividers resistors are equal, so the measured voltage is double the shown voltage.

NRF24l01+ socket
with CE and CSN pins connected to digital pins 7 and 8 ( you use RF24 radio(7, 8); in Arduino code). There is a 4.7uF capacitor connected across Vin and GND of the port
*/

/*
 *    //------------------
   float batteryChargeCurrent = ((float)analogRead(chargeCurrent_PIN) * voltage/1024)/ 3.3 * 250; // CURRENT battery
   if (abs(batteryChargeCurrent  - lastBattCurrent) >= Threshold) {
      lastBattCurrent = batteryChargeCurrent;
      txBattCurrent= true;
   }   
   #ifdef MY_DEBUG
     Serial.print("Charge current ");
     Serial.print(batteryChargeCurrent);
     Serial.println("mA ");
   #endif

 
     byte currentHour = hour();
    byte currentMinute = minute();
    if (currentHour != lastHour) {
        Serial.println( "Resyncing hour" );
        requestTime(); // sync the time every hour
        wait(500 );
        lastHour = currentHour;
     }
     else if (abs(currentMinute-lastMinute) > 5) {      
        #ifdef MY_DEBUG
          Serial.print("Rain Bucket: ");
          Serial.println(rainBucket);
        #endif
        send(rainMsg.set((float)rainBucket / 100, 1));
        rainBucket=0;
        wait(500);
    }

#define bucketSize 30                 //  MI-SOL Rain Guage which has a bucket size of 0.3 mm
unsigned long lastSend;               //  Last Send millis()
unsigned long lastTipTime=millis();
volatile unsigned int rainBucket=0; 
//-------------------------------------------------------------------
void sensorTipped()
{
  unsigned long thisTipTime = millis();
  if (thisTipTime - lastTipTime > 100UL){
    rainBucket += bucketSize; // adds bucket size hundredths of unit each tip
  }
  wait(100);
  #ifdef MY_DEBUG
      Serial.println("Sensor Tipped");
  #endif
  lastTipTime = thisTipTime;
}   
  unsigned long functionTimeout = millis();
  while(!timeReceived && millis() - functionTimeout < 30000UL ) {
    requestTime();
    Serial.println("Getting Time");
    wait(1000);
  }
  lastHour = hour();
  lastMinute = minute();
//------------------------- 
  unsigned long now = millis();
  // If no time has been received yet, request it every 10 second from controller
  if (!timeReceived && (now-lastRequest) > (10UL*1000UL)) {
    requestTime();  
    lastRequest = now;
    }

// This is called when a new time value was received
void receiveTime(unsigned long controllerTime) {
  // Ok, set incoming time
  Serial.println(controllerTime);
  setTime(controllerTime);
  #ifdef MY_DEBUG
      Serial.print("Ora: ");
      Serial.println(hour());
      Serial.print("Minuti: ");
      Serial.println(minute());
  #endif
  timeReceived = true;
}
//-------------------------------------------------------------------
*/
