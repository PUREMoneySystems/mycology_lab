#include <HashMap.h>

#include <mcp2515_defs.h>
#include <defaults.h>
#include <mcp2515.h>
#include <global.h>
#include <Canbus.h>


//Run-level 0 is system sleep, CANBUS can only receive a wakeup message, system independent
//          1 is CANBUS active, all sensors and drivers are off, system independent
//          2 is CANBUS active, sensors active, drivers are off, system independent
//          3 is CANBUS active, sensors active, drivers active, system independent
//          4 is CANBUS active, sensors active, drivers active, system centrally controlled
int currentSystemRunLevel = 1;


//Sensors have UCID's < 100
//Drivers have UCID's >= 100
//Drivers State have UCID's >= 200

unsigned char allDevices_PID = 1; //ID used for message routing.  1 is the PID for all devices

unsigned int temperatureControlBox_CANBUSID = 0x0010; 
unsigned char temperatureControlBox_PID = 10; 
const unsigned char tcb_runlevel_UCID = 100;
const unsigned char tcb_runlevelState_UCID = 200;
const unsigned char tcb_mainPeltierPowerDriver_UCID = 101;
const unsigned char tcb_mainPeltierPowerDriverState_UCID = 201;
const unsigned char tcb_backupPeltierPowerDriver_UCID = 102;
const unsigned char tcb_backupPeltierPowerDriverState_UCID = 202;
const unsigned char tcb_hotTemperatureSensor_UCID = 1;
const unsigned char tcb_hotFanPowerDriver_UCID = 103;
const unsigned char tcb_hotFanPowerDriverState_UCID = 203;
const unsigned char tcb_hotFanSpeedSensor_UCID = 2;
const unsigned char tcb_hotFanSpeedDriver_UCID = 104;
const unsigned char tcb_hotFanSpeedDriverState_UCID = 204;
const unsigned char tcb_coldTemperatureSensor_UCID = 3;
const unsigned char tcb_coldFanPowerDriver_UCID = 105;
const unsigned char tcb_coldFanPowerDriverState_UCID = 205;
const unsigned char tcb_coldFanSpeedSensor_UCID = 4;
const unsigned char tcb_coldFanSpeedDriver_UCID = 106;
const unsigned char tcb_coldFanSpeedDriverState_UCID = 206;

unsigned int humidityControlBox_CANBUSID = 0x0011;  
unsigned char humidityControlBox_PID = 11;  
const unsigned char hcb_runlevel_UCID = 107;
const unsigned char hcb_runlevelState_UCID = 207;
const unsigned char hcb_humidifierPowerDriver_UCID = 108;
const unsigned char hcb_humidifierPowerDriverState_UCID = 208;
const unsigned char hcb_waterReserveSensor_UCID = 5;

unsigned int growthChamber_CANBUSID = 0x0012; 
unsigned char growthChamber_PID = 12;  
const unsigned char gc_runlevel_UCID = 109;
const unsigned char gc_runlevelState_UCID = 209;
const unsigned char gc_temperature1Sensor_UCID = 6;
const unsigned char gc_temperature2Sensor_UCID = 7;
const unsigned char gc_humiditySensor_UCID = 8;

unsigned int fruitChamber_CANBUSID = 0x0013; 
unsigned char fruitChamber_PID = 13;  
const unsigned char fc_runlevel_UCID = 110;
const unsigned char fc_runlevelState_UCID = 210;
const unsigned char fc_lightDriver_UCID = 111;
const unsigned char fc_lightDriverState_UCID = 211;
const unsigned char fc_temperature1Sensor_UCID = 9;
const unsigned char fc_temperature2Sensor_UCID = 10;
const unsigned char fc_humiditySensor_UCID = 11;


//Heartbeat message names.  We will add send interval rules for these messages in setup
char tcbHeartBeatMessageName[ ] = "1200";
char hcbHeartBeatMessageName[ ] = "1207";
char gcHeartBeatMessageName[ ] = "1209";
char fcHeartBeatMessageName[ ] = "1210";

//Temperature Control Box 
char tcbMainPeltierPowerDriverMessageName[ ] = "1201";
char tcbBackupPeltierPowerDriverMessageName[ ] = "1202";
char tcbHotTemperatureMessageName[ ] = "11";
char tcbHotFanPowerDriverMessageName[ ] = "1203";
char tcbHotFanSpeedMessageName[ ] = "12";
char tcbHotFanSpeedDriverMessageName[ ] = "1204";
char tcbColdTemperatureMessageName[ ] = "13";
char tcbColdFanPowerDriverMessageName[ ] = "1205";
char tcbColdFanSpeedMessageName[ ] = "14";
char tcbColdFanSpeedDriverMessageName[ ] = "1206";

//Humidity Control Box 
char hcbHumidifierPowerDriverMessageName[ ] = "1208";
char hcbWaterReserveMessageName[ ] = "15";

//Growth Chamber 
char gcTemperature1MessageName[ ] = "16";
char gcTemperature2MessageName[ ] = "17";
char gcHumidityMessageName[ ] = "18";

//Fruit Chamber 
char fcLightDriverMessageName[ ] = "1211";
char fcTemperature1MessageName[ ] = "19";
char fcTemperature2MessageName[ ] = "110";
char fcHumidityMessageName[ ] = "111";


unsigned char canbus_tx_buffer[8]; //Send buffer
unsigned char canbus_rx_buffer[8]; //Receive buffer

//Log of the times a message was sent for any of the UCID's by this system
const byte timesMessageLastSent_HASHSIZE = 5; 
//byte timesMessageLastSent_nextIndex = 0;
HashType<char*,int> timesMessageLastSent_hashRawArray[timesMessageLastSent_HASHSIZE];
HashMap<char*,int> timesMessageLastSentHashMap = HashMap<char*,int>(timesMessageLastSent_hashRawArray , timesMessageLastSent_HASHSIZE ); 

//The millisecond intervals that UCID's should send a message to the system
int defaultMessageSendInterval = 1; // 1 Second
const byte messageSendInterval_HASHSIZE = 10;
//byte messageSendInterval_nextIndex = 0;
HashType<char*,int> messageSendInterval_hashRawArray[messageSendInterval_HASHSIZE];
HashMap<char*,int> messageSendIntervalHashMap = HashMap<char*,int>(messageSendInterval_hashRawArray , messageSendInterval_HASHSIZE ); 




//NEW CODE TO MANAGE MESSAGE SEND TIMING -- NO HASHMAPs
String timesMessageLastSent_MessageNames[50];
unsigned long timesMessageLastSent_MessageSendTime[50];
byte timesMessageLastSent_nextIndex = 0;
String messageSendInterval_MessageNames[50];
unsigned long messageSendInterval_Interval[50];
byte messageSendInterval_nextIndex = 0;








boolean currentReceiveMessageIsRunlevelChange = false;
boolean currentReceiveMessageIsASensorValue = false;
int currentReceiveMessageDataAsInteger = 0;
float currentReceiveMessageDataAsFloat = 0.0;
unsigned int currentMessageID = 1000;
unsigned char currentMessagePID = 0;
int currentMessageUCID = 0;

String currentSendMessageName;
unsigned long currentSendMessageLastSent = 0;
byte currentSendMessageLastSentIndex = 0;
unsigned long currentSendMessageInterval = defaultMessageSendInterval;
byte currentSendMessageIntervalIndex = 0;
unsigned long currentMilliseconds = millis();
unsigned long currentSendMessageIntervalRange = (currentSendMessageInterval * 2);
unsigned long currentMillisecondsInInterval = (currentMilliseconds % currentSendMessageIntervalRange);




boolean setupCANBUS(){
  //Add some message send intervals.  Heartbeat every 10 seconds
  String tempMessageName;
  //messageSendIntervalHashMap[0](tcbHeartBeatMessageName, 10);
  //messageSendIntervalHashMap[1](hcbHeartBeatMessageName, 10);
  //messageSendIntervalHashMap[2](gcHeartBeatMessageName, 10);
  //messageSendIntervalHashMap[3](fcHeartBeatMessageName, 10); 
  //messageSendInterval_nextIndex = 4;
  
  for(int i = 0; i < messageSendInterval_nextIndex; i++){
   Serial.print(i); Serial.print(" :: "); Serial.print( messageSendIntervalHashMap[i].getHash()); Serial.print(" :: "); Serial.println(messageSendIntervalHashMap[i].getValue());
  }
    
  //Initialize the CANBUS
  Serial.println("CAN BUS starting..."); 
  delay(1000);
  if(Canbus.init(CANSPEED_500)){ /* Initialise MCP2515 CAN controller at the specified speed */
    Serial.println("CAN Init ok");
    return true;
  } else {
    Serial.println("Can't init CAN");
    return false;
  }  
}

boolean readMessageFromCANBUS(){  
  boolean returnMessageReadFromCANBUS = false;
  
  int retryCount = 8000; 
  while(retryCount > 0){
    retryCount--;
      
    unsigned int ID = Canbus.message_rx(canbus_rx_buffer);   
    currentMessagePID = canbus_rx_buffer[0];
  
    if(canbus_rx_buffer[0] != 0 && canbus_rx_buffer[1] != 0 && canbus_rx_buffer[2] != 0){
      Serial.print("Receiving from CANBUS : ");
      for(int i=0; i<8; i++){
        Serial.print(canbus_rx_buffer[i]);
        Serial.print(" ");
      }
      Serial.println("");
    }
    
    if(currentMessagePID > 0 && currentMessagePID < 14){
      currentMessageID = ID;
      returnMessageReadFromCANBUS = true;
      break;
    }
  }
  
  if(!returnMessageReadFromCANBUS){return returnMessageReadFromCANBUS;}
  
  //If the message is for everyone explicitly, or this device specifically, or if we are running in independent control mode then we have a viable message from the CANBUS
  if(currentMessagePID == allDevices_PID || currentMessagePID == temperatureControlBox_PID || (currentMessagePID > 0 && currentSystemRunLevel < 4)){
    
    currentReceiveMessageIsRunlevelChange = (canbus_rx_buffer[1] == 100);
    currentReceiveMessageIsASensorValue = (canbus_rx_buffer[1] < 100 || canbus_rx_buffer[1] >= 200);
    
    Serial.print("Receiving from CANBUS : ");
    for(int i=0; i<8; i++){
      Serial.print(canbus_rx_buffer[i]);
      Serial.print(" ");
    }
    Serial.println("");
  }  
  
  return returnMessageReadFromCANBUS;
}


void sendMessageToCANBUS(unsigned char targetPID, unsigned char ucid, float data){
    //Load the CANBUS Message send interval and time last sent  
    loadCANBUSSendMessage(targetPID, ucid); 
        
    //Make sure this message is overdue on the CANBUS    
    if(!isCurrentSendMessageOverdue()){ 
      return; 
    }else{      
      Serial.println("Overdue...");
    }

    //Load the send buffer with Target PID, UCID, and Data
    canbus_tx_buffer[0] = targetPID; // Receiveing device PID. 1 for all devices
    canbus_tx_buffer[1] = ucid;      // Use Case reporting data / being executed   
   
    //Convert the float to unsigned char
    const size_t conversionBufferSize = 4;
    unsigned char conversionBuffer[conversionBufferSize];
    float *floatPointerToCharArray = (float*)conversionBuffer;                  
    *floatPointerToCharArray = data;
   
    for(int i =0; i < 4; i++){
      canbus_tx_buffer[i+2] = conversionBuffer[i];
    }
    
    Serial.print("Transmitting on CANBUS : ");
    for(int i=0; i<8; i++){
      Serial.print(canbus_tx_buffer[i]);
      Serial.print(" ");
    }
    Serial.println("");
    //Serial.print("Send Message Stats : ");
    //Serial.print(messageSendInterval_nextIndex);Serial.print(" ");
    //Serial.print(timesMessageLastSent_nextIndex);Serial.print(" ");
    //Serial.println(currentMillisecondsInInterval);
    
    
    Canbus.message_tx(temperatureControlBox_CANBUSID, canbus_tx_buffer); // Send the message on the CAN Bus to be picked up by the other devices
        
    //Log the time of the message being sent to the CANBUS
    updateCurrentSendMessageTime();
               
}

void loadCANBUSSendMessage(unsigned char targetPID, unsigned char ucid){  
  getSendMessageName(targetPID, ucid, currentSendMessageName);
  Serial.print(sizeof(currentSendMessageName)); Serial.print(" - ");
  for(int i = 0; i < sizeof(currentSendMessageName); i++){
    Serial.print(currentSendMessageName[i]);Serial.print(".");
  }
  Serial.print(" -- ");
  
  currentSendMessageInterval = defaultMessageSendInterval;
  currentSendMessageIntervalIndex = messageSendIntervalHashMap.getIndexOf(currentSendMessageName);
  Serial.print(currentSendMessageIntervalIndex);Serial.print(" ");
  if(currentSendMessageIntervalIndex < messageSendInterval_nextIndex || currentSendMessageIntervalIndex == 0){
    //Serial.println("This message is saying a specific interval was configured in the HashMap.");
    currentSendMessageInterval = messageSendIntervalHashMap.getValueOf(currentSendMessageName);
    Serial.println(currentSendMessageInterval);
  }
    
  currentSendMessageLastSent = 0;
  currentSendMessageLastSentIndex = timesMessageLastSentHashMap.getIndexOf(currentSendMessageName);
  //Serial.print(currentSendMessageLastSentIndex);Serial.print(" ");
  if(currentSendMessageLastSentIndex < timesMessageLastSent_nextIndex || currentSendMessageLastSentIndex == 0){
    //Serial.println("This message is saying a specific time sent was previously recorded in the HashMap.");
    currentSendMessageLastSent = timesMessageLastSentHashMap.getValueOf(currentSendMessageName);
  }
  
  //Get the current millisecond count in the range (double the interval)...if larger than interval then overdue
  currentMilliseconds = millis();
  currentSendMessageIntervalRange = ((currentSendMessageInterval * 1000) * 2);
  currentMillisecondsInInterval = (currentMilliseconds % currentSendMessageIntervalRange);
}

void getSendMessageName(unsigned char targetPID, unsigned char ucid, String messageName){ 
  //FIX THIS....can't figure out how to convert String to char* so that HashMap.getIndexOf(char*) resolves correctly
  /*
  if(targetPID == allDevices_PID){
    switch(ucid){
      case tcb_runlevelState_UCID:
        messageName = tcbHeartBeatMessageName;
        break;
      case hcb_runlevelState_UCID:
        messageName = hcbHeartBeatMessageName;
        break;
      case gc_runlevelState_UCID:
        messageName = gcHeartBeatMessageName;
        break;
      case fc_runlevelState_UCID:
        messageName = fcHeartBeatMessageName;
        break;
      case tcb_hotTemperatureSensor_UCID:
        messageName = fcHeartBeatMessageName;
        break;
      case tcb_hotFanSpeedSensor_UCID:
        messageName = fcHeartBeatMessageName;
        break;
      case tcb_coldTemperatureSensor_UCID:
        messageName = fcHeartBeatMessageName;
        break;
      case tcb_coldFanSpeedSensor_UCID:
        messageName = fcHeartBeatMessageName;
        break;
    }
  }else{ 
  */    
    String composedMessageName = "";
    composedMessageName += String(targetPID, DEC) + String(ucid, DEC);
    messageName = composedMessageName;    
        
    //int messageNameLength = composedMessageName.length() + 1;
    //composedMessageName.toCharArray(messageName, messageNameLength);  
    //Serial.println(messageName);
  //}
  /*
  
  char returnMessageName[messageNameLength];  
  //returnMessageName[sizeof(returnMessageName) - 1] = '\0';
  Serial.print(sizeof(returnMessageName)); Serial.print(" - ");
  for(int i = 0; i < sizeof(returnMessageName); i++){
    Serial.print(returnMessageName[i], HEX);Serial.print(".");
  }
  Serial.println("");
  char sample[ ] = "1209";
  Serial.print(sizeof("1209")); Serial.print(" - ");
  for(int i = 0; i < sizeof("1209"); i++){
    Serial.print(("1209")[i], HEX);Serial.print(".");
  }
  Serial.println("");
  
  Serial.print(returnMessageName);Serial.print(" ");
  Serial.print(composedMessageName);Serial.println(" ");
  
  if(strcmp("1209", returnMessageName) == 0){
    Serial.println("They are equal");
  }else{
    Serial.println("They are not equal");
  }
    
  
  if(ucid == 209){
    strncpy(messageName, returnMessageName, sizeof(messageName));
  }else{
    strncpy(messageName, "00000", sizeof(messageName));
  }
  */
}

boolean isCurrentSendMessageOverdue(){   
  Serial.print(currentSendMessageLastSent); Serial.print(" - "); Serial.print(currentMillisecondsInInterval);Serial.print(" - "); Serial.println(currentSendMessageInterval);
  if((currentSendMessageLastSent * 1000) > currentMillisecondsInInterval){
    return (((currentSendMessageIntervalRange - (currentSendMessageLastSent * 1000)) + currentMillisecondsInInterval) > (currentSendMessageInterval * 1000));
  }else{
    return ((currentMillisecondsInInterval - (currentSendMessageLastSent * 1000)) > (currentSendMessageInterval * 1000));    
  }
}

void updateCurrentSendMessageTime(){  
  if(currentSendMessageLastSentIndex < timesMessageLastSent_nextIndex || currentSendMessageLastSentIndex == 0){
    //Serial.println("This message is saying a specific time sent was previously recorded in the HashMap....lets update it");
    timesMessageLastSentHashMap[currentSendMessageLastSentIndex].setValue((currentMillisecondsInInterval/1000));
  }else{
    //Serial.println("This message has no specific time sent recorded in the HashMap....lets add it");
    timesMessageLastSentHashMap[timesMessageLastSent_nextIndex](currentSendMessageName, (currentMillisecondsInInterval/1000));
    timesMessageLastSent_nextIndex++;
  }
}


unsigned int MY_ID = growthChamber_CANBUSID;
unsigned char MY_PID = growthChamber_PID;








#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

#define RHT22_PIN 2




int humiditySensorPin = 2;
int temperatureSensorPin = 3;

int currentTemperature1Reading;
float currentTemperature1ReadingInMilliVolts = 0;
float currentTemperature1C = 0;

unsigned int currentTemperature2Reading;
float currentTemperature2C = 0;
unsigned int currentHumidityReading;
float currentHumidity;



char ssid[] = "TPAHKRSPC";     //  your network SSID (name)
char pass[] = "QRTjj7C@m";    // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status





const char* mltcrDeviceID = "vE3BF8B25B5F806A";  //Mycology lab - Temperature Controller Reading



//Celsius to Fahrenheit conversion
double CtoF(double celsius){ return 1.8 * celsius + 32; }

// fast integer version with rounding
int C2F(int celsius){ return (celsius * 18 + 5)/10 + 32; }

//Celsius to Kelvin conversion
double CtoK(double celsius){ return celsius + 273.15; }



// dewPoint function NOAA
// reference (1) : http://wahiduddin.net/calc/density_algorithms.htm
// reference (2) : http://www.colorado.edu/geography/weather_station/Geog_site/about.htm
double dewPoint(double celsius, double humidity){
	// (1) Saturation Vapor Pressure = ESGG(T)
	double RATIO = 373.15 / (273.15 + celsius);
	double RHS = -7.90298 * (RATIO - 1);
	RHS += 5.02808 * log10(RATIO);
	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
	RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
	double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
	double T = log(VP/0.61078);   // temp var
	return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 6.9 x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFast(double celsius, double humidity){
	double a = 17.271;
	double b = 237.7;
	double temp = (a * celsius) / (b + celsius) + log(humidity*0.01);
	double Td = (b * temp) / (a - temp);
	return Td;
}





// the setup routine runs once when you press reset:
void setup()  {   
  Serial.begin(115200);
  Serial.println("DHT22 TEST PROGRAM ");
  Serial.println();
  Serial.println("Type,\tstatus,\tHumidity (%),\tTemperature (C)");
    
  //Set up the Temp sensor
  pinMode(temperatureSensorPin, INPUT);
    

  //Attempt to connect using WPA2 encryption:
  Serial.println("Attempting to connect to WPA network...");
  status = WiFi.begin(ssid, pass);

  // if you're not connected, stop here:
  if ( status != WL_CONNECTED) {
    Serial.println("Couldn't get a wifi connection");
    while(true);
  }
  // if you are connected, print out info about the connection:
  else {
    Serial.println("Connected to network");
  }
      
  
  //Connect to the CANBUS
  setupCANBUS(); 
  
} 




void loop()  {      
    
  //Receive any CANBUS message
  boolean messageReceived = readMessageFromCANBUS();
  
  //Check if the message is a request to change runlevels....if so, change the runlevel and quit
  if(messageReceived && currentReceiveMessageIsRunlevelChange && (currentSystemRunLevel != currentReceiveMessageDataAsInteger)){
     currentSystemRunLevel = currentReceiveMessageDataAsInteger;
     return;
  }
  
  if(currentSystemRunLevel > 0){    
    //Notify all devices on the CANBUS that this device is awake with the current runlevel
    sendMessageToCANBUS(allDevices_PID, gc_runlevelState_UCID, currentSystemRunLevel);
    
    if(currentSystemRunLevel > 1){ 
      //Read all sensor values
      boolean sensorReadSuccess = readAllSensors();
      if(sensorReadSuccess){          
        //Send sensor values and driver states to the CANBUS
        sendMessageToCANBUS(allDevices_PID, gc_temperature1Sensor_UCID, currentTemperature1C);
        sendMessageToCANBUS(allDevices_PID, gc_temperature2Sensor_UCID, currentTemperature2C);
        sendMessageToCANBUS(allDevices_PID, gc_humiditySensor_UCID, currentHumidity);
      }         
    }
  }         
}



boolean readAllSensors(){  
  //Report Temperature Sensor   
  currentTemperature1Reading = analogRead(temperatureSensorPin);  
  currentTemperature1ReadingInMilliVolts = (currentTemperature1Reading * 5.0) / 1024;
  currentTemperature1C = (currentTemperature1ReadingInMilliVolts -0.5) * 100; //converting from 10 mv per degree with 500 mV offset
    
  //Report Humidity/Temperature Sensor    
  boolean success = readRHT03();
  if (success) {
    currentHumidity = (currentHumidityReading/10.0);
    currentTemperature2C = (currentTemperature2Reading/10.0);    
  }    
  
  return success;
}








boolean readRHT03(){
 unsigned int rht22_timings[88];
 initRHT03();
 
 pinMode(RHT22_PIN, OUTPUT);
 digitalWrite(RHT22_PIN, LOW);
 delayMicroseconds(3000);
 
 digitalWrite(RHT22_PIN, HIGH);
 delayMicroseconds(30);
 pinMode(RHT22_PIN, INPUT);
 
 int state=digitalRead(RHT22_PIN);
 unsigned int counter=0;
 unsigned int signalLineChanges=0;
 TCNT1=0;
 cli();
 while (counter!=0xffff) {
   counter++;
   if(state!=digitalRead(RHT22_PIN))   {
     state=digitalRead(RHT22_PIN);
     rht22_timings[signalLineChanges] = TCNT1;
     TCNT1=0;
     counter=0;
     signalLineChanges++;
   }
   if(signalLineChanges==83)
   break;
 }
 sei();
 boolean errorFlag=false;
 if (signalLineChanges != 83) {
   currentTemperature2Reading=-1;
   currentHumidityReading=-1;
   errorFlag=true;
 } else {
   errorFlag=!(getHumidityAndTemp(rht22_timings));
 }
 
 return !errorFlag;
}


void initRHT03(){
 //for (int i = 0; i < 86; i++) { rht22_timings[i] = 0; } 
 TCCR1A=0;
 TCCR1B=_BV(CS10);
 pinMode(RHT22_PIN,INPUT);
 digitalWrite(RHT22_PIN,HIGH); 
}



// DEBUG routine: dump timings array to serial
void debugPrintTimings(unsigned int rht22_timings[]) { // XXX DEBUG
for (int i = 0; i < 88; i++) { // XXX DEBUG
 if (i%10==0) { Serial.print("\n\t"); }
 char buf[24];
 sprintf(buf, i%2==0 ? "H[%02i]: %-3i " : "L[%02i]: %-3i ", i, rht22_timings[i]);
 Serial.print(buf);
 } // XXX DEBUG
 Serial.print("\n"); // XXX DEBUG
}


boolean getHumidityAndTemp(unsigned int rht22_timings[]){
  // 25us = 400, 70us = 1120;
  currentHumidityReading=0;
  for(int i=0;i<32;i+=2)  {
    if(rht22_timings[i+3]>750){ 
      currentHumidityReading|=(1 << (15-(i/2)));
    }
  }
   
  currentTemperature2Reading=0;
  for(int i=0;i<32;i+=2) { 
    if(rht22_timings[i+35]>750)
      currentTemperature2Reading|=(1<<(15-(i/2)));
  }
 
  int cksum=0;
  for(int i=0;i<16;i+=2) { 
    if(rht22_timings[i+67]>750)
      cksum|=(1<<(7-(i/2)));
  }
 
  int cChksum=((currentHumidityReading >> 8)+(currentHumidityReading & 0xff) + (currentTemperature2Reading >> 8) + (currentTemperature2Reading &0xff)) & 0xFF; 
   
  if(currentTemperature2Reading & 0x8000)
    currentTemperature2Reading=(currentTemperature2Reading & 0x7fff)*-1;
     
  if(cChksum == cksum)
    return true;
   
  return false;
}


void printMacAddress() {
  // the MAC address of your Wifi shield
  byte mac[6];                    

  // print your MAC address:
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
}

void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  byte numSsid = WiFi.scanNetworks();

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet<numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.print("\tEncryption: ");
    Serial.println(WiFi.encryptionType(thisNet));
  }
}




