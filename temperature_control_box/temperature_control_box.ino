
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
int currentSystemRunLevel = 2;


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




unsigned char canbus_tx_buffer[8]; //Send buffer
unsigned char canbus_rx_buffer[8]; //Receive buffer




boolean currentReceiveMessageIsRunlevelChange = false;
boolean currentReceiveMessageIsASensorValue = false;
int currentReceiveMessageDataAsInteger = 0;
float currentReceiveMessageDataAsFloat = 0.0;
unsigned int currentReceiveMessageID = 1000;
unsigned char currentReceiveMessagePID = 0;
int currentReceiveMessageUCID = 0;



//NEW CODE TO MANAGE MESSAGE SEND TIMING -- NO HASHMAPs
char timesMessageLastSent_MessageNames[50][5];
unsigned long timesMessageLastSent_MessageSendTime[50];
byte timesMessageLastSent_nextIndex = 0;
char messageSendInterval_MessageNames[50][5];
unsigned long messageSendInterval_Interval[50];
byte messageSendInterval_nextIndex = 0;
unsigned long defaultMessageSendInterval = 1000; // 1 Second


char currentSendMessageName[5];
unsigned long currentSendMessageLastSent = 0;
byte currentSendMessageLastSentIndex = 0;
unsigned long currentSendMessageInterval = defaultMessageSendInterval;
byte currentSendMessageIntervalIndex = 0;
unsigned long currentMilliseconds = millis();
unsigned long currentSendMessageIntervalRange = (currentSendMessageInterval * 2);
unsigned long currentMillisecondsInInterval = (currentMilliseconds % currentSendMessageIntervalRange);




boolean setupCANBUS(){
  //Add some message send intervals.  Heartbeat every 10 seconds
  char tempMessageName[5];
  getSendMessageName(allDevices_PID, tcb_runlevelState_UCID, tempMessageName);
  messageSendInterval_nextIndex = addMessageSendTime(tempMessageName, 10000, messageSendInterval_nextIndex, messageSendInterval_MessageNames, messageSendInterval_Interval);

  getSendMessageName(allDevices_PID, hcb_runlevelState_UCID, tempMessageName);
  messageSendInterval_nextIndex = addMessageSendTime(tempMessageName, 10000, messageSendInterval_nextIndex, messageSendInterval_MessageNames, messageSendInterval_Interval);

  getSendMessageName(allDevices_PID, gc_runlevelState_UCID, tempMessageName);
  messageSendInterval_nextIndex = addMessageSendTime(tempMessageName, 10000, messageSendInterval_nextIndex, messageSendInterval_MessageNames, messageSendInterval_Interval);

  getSendMessageName(allDevices_PID, fc_runlevelState_UCID, tempMessageName);
  messageSendInterval_nextIndex = addMessageSendTime(tempMessageName, 10000, messageSendInterval_nextIndex, messageSendInterval_MessageNames, messageSendInterval_Interval);

  //for(byte i = 0; i < messageSendInterval_nextIndex; i++){
  // Serial.print(i); Serial.print(" :: "); Serial.print(messageSendInterval_MessageNames[i]); Serial.print(" :: "); Serial.println(messageSendInterval_Interval[i]);
  //}
    
  //Initialize the CANBUS
  Serial.print("CANBUS starting..."); 
  delay(1000);
  if(Canbus.init(CANSPEED_500)){ /* Initialise MCP2515 CAN controller at the specified speed */
    Serial.println("CANBUS Init ok");
    return true;
  } else {
    Serial.println("Can't init CANBUS");
    return false;
  }  
}

boolean readMessageFromCANBUS(){  
  boolean returnMessageReadFromCANBUS = false;
  
  int retryCount = 8000; 
  while(retryCount > 0){
    retryCount--;
      
    unsigned int ID = Canbus.message_rx(canbus_rx_buffer); 
  
    if(canbus_rx_buffer[0] != currentReceiveMessagePID || canbus_rx_buffer[1] != currentReceiveMessageUCID){      
      if(canbus_rx_buffer[0] > 0 && canbus_rx_buffer[0] < 14){
        Serial.print("Receiving from CANBUS : ");
        for(int i=0; i<8; i++){
          Serial.print(canbus_rx_buffer[i]);
          Serial.print(" ");
        }
        Serial.println("");
        
        currentReceiveMessageID = ID;  
        currentReceiveMessagePID = canbus_rx_buffer[0];
        currentReceiveMessageUCID = canbus_rx_buffer[1];
        returnMessageReadFromCANBUS = true;
        break;
      }
    }    
  }
  
  if(!returnMessageReadFromCANBUS){return returnMessageReadFromCANBUS;}
  
  //If the message is for everyone explicitly, or this device specifically, or if we are running in independent control mode then we have a viable message from the CANBUS
  if(currentReceiveMessagePID == allDevices_PID || currentReceiveMessagePID == temperatureControlBox_PID || (currentReceiveMessagePID > 0 && currentSystemRunLevel < 4)){
    
    currentReceiveMessageIsRunlevelChange = (canbus_rx_buffer[1] == tcb_runlevel_UCID || 
                                              canbus_rx_buffer[1] == hcb_runlevel_UCID || 
                                              canbus_rx_buffer[1] == gc_runlevel_UCID || 
                                              canbus_rx_buffer[1] == fc_runlevel_UCID);
    currentReceiveMessageIsASensorValue = (canbus_rx_buffer[1] < 100 || canbus_rx_buffer[1] >= 200);
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
      //Serial.println("Overdue...");
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
  //for(int i = 0; i < 5; i++){
  //  Serial.print(currentSendMessageName[i]);Serial.print(".");
  //}
  //Serial.print(" -- ");
  
  currentSendMessageInterval = defaultMessageSendInterval;
  currentSendMessageIntervalIndex = getIndexOfSendInterval(targetPID, ucid, messageSendInterval_MessageNames);
  //Serial.print(currentSendMessageIntervalIndex);Serial.print(" ");
  if(currentSendMessageIntervalIndex < messageSendInterval_nextIndex || currentSendMessageIntervalIndex == 0){
    //Serial.print("This message is saying a specific interval was configured in the HashMap. - ");
    currentSendMessageInterval = messageSendInterval_Interval[currentSendMessageIntervalIndex];
    //Serial.println(currentSendMessageInterval);
  }
    
  currentSendMessageLastSent = 0;
  currentSendMessageLastSentIndex = getIndexOfSendInterval(targetPID, ucid, timesMessageLastSent_MessageNames);
  //Serial.print(currentSendMessageLastSentIndex);Serial.print(" ");
  if(currentSendMessageLastSentIndex < timesMessageLastSent_nextIndex || currentSendMessageLastSentIndex == 0){
    //Serial.println("This message is saying a specific time sent was previously recorded in the HashMap.");
    currentSendMessageLastSent = timesMessageLastSent_MessageSendTime[currentSendMessageLastSentIndex];
  }
  
  //Get the current millisecond count in the range (double the interval)...if larger than interval then overdue
  currentMilliseconds = millis();
  currentSendMessageIntervalRange = ((currentSendMessageInterval * 1000) * 2);
  currentMillisecondsInInterval = (currentMilliseconds % currentSendMessageIntervalRange);
}

void getSendMessageName(unsigned char targetPID, unsigned char ucid, char* sendMessageName){  
  String composedMessageName; 
  composedMessageName = String(targetPID, DEC) + String(ucid, DEC);
  int messageNameLength = composedMessageName.length() + 1;
  composedMessageName.toCharArray(sendMessageName, messageNameLength);
}


byte getIndexOfSendInterval(unsigned char targetPID, unsigned char ucid, char messageNames[][5]){
  byte returnIndex = 255;
  char tempMessageName[5];
  getSendMessageName(targetPID, ucid, tempMessageName);
    
  for(byte i=0; i< 50; i++){
    //Serial.print("getIndexOfSendInterval - ");Serial.print(messageNames[i]);Serial.print(".");Serial.println(tempMessageName);
    if(strcmp(messageNames[i], tempMessageName) == 0){
      returnIndex = i;
      break;
    }
  }
  
  return returnIndex;
}

byte addMessageSendTime(char messageName[], unsigned long sendTime, byte nextIndex, char messageNames[][5], unsigned long messageTimes[]){
  memcpy(messageNames[nextIndex], messageName, 5);
  messageTimes[nextIndex] = sendTime;
  nextIndex++;
  return nextIndex;
}


boolean isCurrentSendMessageOverdue(){   
  //Serial.print(currentSendMessageLastSent); Serial.print(" - "); Serial.print(currentMillisecondsInInterval);Serial.print(" - "); Serial.println(currentSendMessageInterval);
  if(currentSendMessageLastSent > currentMillisecondsInInterval){
    return (((currentSendMessageIntervalRange - currentSendMessageLastSent) + currentMillisecondsInInterval) > currentSendMessageInterval);
  }else{
    return ((currentMillisecondsInInterval - currentSendMessageLastSent) > currentSendMessageInterval);    
  }
}

void updateCurrentSendMessageTime(){  
  if(currentSendMessageLastSentIndex < timesMessageLastSent_nextIndex || currentSendMessageLastSentIndex == 0){
    //Serial.println("This message is saying a specific time sent was previously recorded in the HashMap....lets update it");
    timesMessageLastSent_MessageSendTime[currentSendMessageLastSentIndex] = currentMillisecondsInInterval;
  }else{
    //Serial.println("This message has no specific time sent recorded in the HashMap....lets add it");
    memcpy(timesMessageLastSent_MessageNames[timesMessageLastSent_nextIndex], currentSendMessageName, 5);
    timesMessageLastSent_MessageSendTime[timesMessageLastSent_nextIndex] = currentMillisecondsInInterval;
    timesMessageLastSent_nextIndex++;
  }
}



unsigned int MY_ID = temperatureControlBox_CANBUSID;
unsigned char MY_PID = temperatureControlBox_PID;






#include <PWM.h>

    
int mainPeltierPowerDriverPin = 4;
int backupPeltierPowerDriverPin = 5;

int hotTemperatureSensorPin = 1;      // Analog Arduino Pin used to determine temperature
int hotFanPowerDriverPin = 6;         // Digital Arduino Pin used to control fan power (relay)
int hotFanSpeedSensorPin = 13;         // Digital Arduino Pin used to determine fan speed (RPM)
int hotFanSpeedDriverPin = 11;         // Analog Arduino Pin used to control fan speed (PWM)

int coldTemperatureSensorPin = 12;     // Analog Arduino Pin used to determine temperature
int coldFanPowerDriverPin = 7;        // Digital Arduino Pin used to control fan power (relay)
int coldFanSpeedSensorPin = A2;        // Analog Arduino Pin used to determine fan speed (RPM)
int coldFanSpeedDriverPin = 10;        // Analog Arduino Pin used to control fan speed (PWM)













//Debug variables
unsigned long lastFanSpeedAdjustTime = 0;






//Current Peltier Driver values
boolean currentMainPeltierPowerOn = false;
boolean currentBackupPeltierPowerOn = false;

//Current Hot Temperature Sensor values  
int currentHotTemperatureReading = 0;  
float currentHotTemperatureReadingInMilliVolts = 0;
float currentHotTemperatureC = 0;

//Hot fan frequency and current power / speed values
int32_t hotFanPWMFrequency = 25000;       //frequency in Hz (25kHz)
int currentHotFanSpeedDriverValue = 0;
float currentHotFanSpeedReading = 0.0;
boolean currentHotFanPowerOn = false;
 
//Current Cold Temperature Sensor values
int currentColdTemperatureReading = 0;  
float currentColdTemperatureReadingInMilliVolts = 0;
float currentColdTemperatureC = 0; 

//Cold fan frequency and current power / speed values
int32_t coldFanPWMFrequency = 25000;       //frequency in Hz (25kHz)
int currentColdFanSpeedDriverValue = 0;
float currentColdFanSpeedReading = 0.0;
boolean currentColdFanPowerOn = false;
 



boolean driverEnabled(){
  return (currentMainPeltierPowerOn || currentBackupPeltierPowerOn || currentHotFanPowerOn || currentColdFanPowerOn);
}

void disableAllDrivers(){  
  currentMainPeltierPowerOn = changePowerDriver(mainPeltierPowerDriverPin, LOW);
  currentBackupPeltierPowerOn = changePowerDriver(backupPeltierPowerDriverPin, LOW); 
  
  currentHotFanPowerOn = changePowerDriver(hotFanPowerDriverPin, LOW);
  currentHotFanSpeedDriverValue = 0.0;
  pwmWrite(hotFanSpeedDriverPin, currentHotFanSpeedDriverValue);
  
  currentColdFanPowerOn = changePowerDriver(coldFanPowerDriverPin, LOW); 
  currentColdFanSpeedDriverValue = 0.0;
  pwmWrite(coldFanSpeedDriverPin, currentColdFanSpeedDriverValue);
}

boolean changePowerDriver(int pin, int newPowerValue){
  digitalWrite(pin, newPowerValue);
  return (newPowerValue == HIGH);  
}















volatile byte coldFanRevolutions = 0;
volatile byte hotFanRevolutions = 0;

void countColdFanRevolutions(){
 coldFanRevolutions++;
}
void countHotFanRevolutions(){
 hotFanRevolutions++;
}

















void setup()  {   
  Serial.begin(9600);
  
  //Set up Peltier Power drivers
  pinMode(mainPeltierPowerDriverPin, OUTPUT);
  pinMode(backupPeltierPowerDriverPin, OUTPUT);  
  
  //Set up Hot Fan Power, Speed Control, and RPM sensors
  pinMode(hotFanPowerDriverPin, OUTPUT);
  pinMode(hotFanSpeedSensorPin, INPUT);
  digitalWrite(hotFanSpeedSensorPin, HIGH);
  
  //Set up the Hot Temperature sensor
  pinMode(hotTemperatureSensorPin, INPUT);
  
  //Set up Cold Fan Power, Speed Control, and RPM sensors
  pinMode(coldFanPowerDriverPin, OUTPUT);
  pinMode(coldFanSpeedSensorPin, INPUT);  
  //attachInterrupt(coldFanSpeedInterrupt, countColdFanRevolutions, RISING);
  digitalWrite(coldFanSpeedSensorPin, HIGH);
  
  //Set up the Cold Temperature sensor
  pinMode(coldTemperatureSensorPin, INPUT);
  
  //Configure and prepare the timers (Timer1 and Timer2 (not Timer0))
  InitTimersSafe();
    
  //Connect to the CANBUS
  setupCANBUS(); 
    
   
  boolean success = SetPinFrequencySafe(coldFanSpeedDriverPin, coldFanPWMFrequency);
  if(success){
    Serial.println("Successfully set the pin frequency...turning on the cold fan");    
    //digitalWrite(coldFanPowerDriverPin, HIGH);    
  }else{    
    Serial.println("Failed to set the pin frequency");
  
    Serial.println(GetPinResolution(0));
    Serial.println(GetPinResolution(1));
    Serial.println(GetPinResolution(2));
    Serial.println(GetPinResolution(3));
    Serial.println(GetPinResolution(4));
    Serial.println(GetPinResolution(5));
    Serial.println(GetPinResolution(6));
    Serial.println(GetPinResolution(7));
    Serial.println(GetPinResolution(8));
    Serial.println(GetPinResolution(9));
    Serial.println(GetPinResolution(10));
    Serial.println(GetPinResolution(11));
  }
  
} 




void loop()  { 
  //Receive any CANBUS message
  //Serial.println("Receiving messages from the CANBUS"); 
  boolean messageReceived = readMessageFromCANBUS();
  
  //Check if the message is a request to change runlevels....if so, change the runlevel and quit
  if(messageReceived && currentReceiveMessageIsRunlevelChange && (currentSystemRunLevel != currentReceiveMessageDataAsInteger)){
     currentSystemRunLevel = currentReceiveMessageDataAsInteger;
     return;
  }
     
  //Drivers are only enabled in Runlevel 3 and 4....disable otherwise
  if(currentSystemRunLevel < 3 && driverEnabled()){ 
      disableAllDrivers();
  }
  
  if(currentSystemRunLevel > 0){    
    //Notify all devices on the CANBUS that this device is awake with the current runlevel
    sendMessageToCANBUS(allDevices_PID, tcb_runlevelState_UCID, currentSystemRunLevel);
    
    if(currentSystemRunLevel > 1){ 
      //Read all sensor values
      boolean sensorReadSuccess = readAllSensors();
      if(sensorReadSuccess){          
        //Send sensor values and driver states to the CANBUS
        sendMessageToCANBUS(growthChamber_PID, tcb_hotTemperatureSensor_UCID, currentHotTemperatureC);
        delay(2000);
        sendMessageToCANBUS(growthChamber_PID, tcb_hotFanSpeedSensor_UCID, currentHotFanSpeedReading);
        delay(2000);
        sendMessageToCANBUS(growthChamber_PID, tcb_coldTemperatureSensor_UCID, currentColdTemperatureC);
        delay(2000);
        sendMessageToCANBUS(growthChamber_PID, tcb_coldFanSpeedSensor_UCID, currentColdFanSpeedReading);
      }
      /*   
      //If we received a sensor value from CANBUS and we are at Runlevel 3, run any independent control rules to respond to its current value
      if(currentSystemRunLevel == 3){      
        //Run independent control rules based on min/max values for sensors
        if(currentHotTemperatureC < 15){
        }
        
        
        
        if(currentReceiveMessageIsASensorValue){            
          if(currentReceiveMessagePID == humidityControlBox_PID && currentReceiveMessageUCID == hcb_waterReserveSensor_UCID){
              Serial.println("Received a Humidity Control Box - Water Reserve Sensor value");
          }
          if(currentReceiveMessagePID == growthChamber_PID && currentReceiveMessageUCID == gc_temperature1Sensor_UCID){
              Serial.println("Received a Grow Chamber - Temperature Sensor 1 value");
          }
          if(currentReceiveMessagePID == growthChamber_PID && currentReceiveMessageUCID == gc_temperature2Sensor_UCID){
              Serial.println("Received a Grow Chamber - Temperature Sensor 2 value");
          }
          if(currentReceiveMessagePID == growthChamber_PID && currentReceiveMessageUCID == gc_humiditySensor_UCID){
              Serial.println("Received a Grow Chamber - Humidity Sensor value");
          }
          if(currentReceiveMessagePID == fruitChamber_PID && currentReceiveMessageUCID == fc_temperature1Sensor_UCID){
              Serial.println("Received a Fruit Chamber - Temperature Sensor 1 value");
          }
          if(currentReceiveMessagePID == fruitChamber_PID && currentReceiveMessageUCID == fc_temperature2Sensor_UCID){
              Serial.println("Received a Fruit Chamber - Temperature Sensor 2 value");
          }
          if(currentReceiveMessagePID == fruitChamber_PID && currentReceiveMessageUCID == fc_humiditySensor_UCID){
              Serial.println("Received a Fruit Chamber - Humidity Sensor value");
          }
        }
      }
      
      //If we received a driver command value from CANBUS and we are at Runlevel 4, execute the driver command according to the central control rule
      if(!currentReceiveMessageIsASensorValue && (currentSystemRunLevel == 4)){            
        switch(currentReceiveMessageUCID){
          case tcb_mainPeltierPowerDriver_UCID:
            Serial.println("Received a Temperature Control Box - Main Peltier Driver Command");
            break;            
          case tcb_backupPeltierPowerDriver_UCID:
            Serial.println("Received a Temperature Control Box - Backup Peltier Driver Command");
            break;            
          case tcb_hotFanPowerDriver_UCID:
            Serial.println("Received a Temperature Control Box - Hot Fan Power Driver Command");
            break;            
          case tcb_hotFanSpeedDriver_UCID:
            Serial.println("Received a Temperature Control Box - Hot Fan Speed Driver Command");
            break;            
          case tcb_coldFanPowerDriver_UCID:
            Serial.println("Received a Temperature Control Box - Cold Fan Power Driver Command");
            break;            
          case tcb_coldFanSpeedDriver_UCID:
            Serial.println("Received a Temperature Control Box - Cold Fan Speed Driver Command");
            break;            
          default:
            Serial.println("Received a Driver Command....no control rule to handle this one");
            break; 
        }       
      } 
      */
      //Clear the CANBUS 
      //delay(1000);
      //sendMessageToCANBUS(256, 256, 256);
    }
  }
    
  
    //Adjust the cold fan speed
    if(lastFanSpeedAdjustTime == 0 || ((lastFanSpeedAdjustTime < (millis() - 10000)) && (millis() > 10000))){
      lastFanSpeedAdjustTime = millis();
      
      if(currentColdFanSpeedDriverValue + 25 > 255){
        currentColdFanSpeedDriverValue = 1;
      }else{
        currentColdFanSpeedDriverValue += 25;
      }
      
      Serial.print("SET cold fan speed to "); Serial.println(currentColdFanSpeedDriverValue);
      pwmWrite(coldFanSpeedDriverPin, currentColdFanSpeedDriverValue);
      readPulse(coldFanSpeedSensorPin);
      
    } 
}



boolean readAllSensors(){
  
    //Report Hot Temperature Sensor    
    currentHotTemperatureReading = analogRead(hotTemperatureSensorPin);  
    currentHotTemperatureReadingInMilliVolts = (currentHotTemperatureReading * 5.0) / 1024;
    currentHotTemperatureC = (currentHotTemperatureReadingInMilliVolts -0.5) * 100; //converting from 10 mv per degree with 500 mV offset
    //Serial.print("Hot ");Serial.print(currentHotTemperatureReadingInMilliVolts); Serial.print(" - "); Serial.println(currentHotTemperatureC);
    
    //Report Hot Fan Speed Sensor
    
    
    
    
    //Report Cold Temperature Sensor
    currentColdTemperatureReading = analogRead(coldTemperatureSensorPin);  
    currentColdTemperatureReadingInMilliVolts = (currentColdTemperatureReading * 5.0) / 1024;
    currentColdTemperatureC = (currentColdTemperatureReadingInMilliVolts -0.5) * 100; //converting from 10 mv per degree with 500 mV offset
    //Serial.print("Cold ");Serial.print(currentColdTemperatureReadingInMilliVolts); Serial.print(" - "); Serial.println(currentColdTemperatureC);
    
    //Report Cold Fan Speed Sensor    
    //readPulse(coldFanSpeedSensorPin);
    //readFanRPMSensor(coldFanSpeedSensorPin);
    
    return true;
}




void readPulse(int speedSensorPin) {  
   
  
  /*
  signed long currentPulse = 0; 
  signed long cumulativePulseAverage = 0;
  signed long cumulativePulseRemainder = 0;
  signed long addendum = 0;
  
  for(int i=1; i<101; i++){
    currentPulse = 0;
    while(currentPulse < 2){
      //Serial.print(currentPulse);
      currentPulse = pulseIn(speedSensorPin, HIGH);
    }
    //Serial.print(currentPulse);Serial.print(" ");
    addendum = currentPulse - cumulativePulseAverage + cumulativePulseRemainder;
    cumulativePulseAverage += addendum / i;
    cumulativePulseRemainder = addendum % i;
  }
  */
  //Serial.println(cumulativePulseAverage);
  
  //double frequency = 1000000/cumulativePulseAverage;
  long frequency = getFrequency(speedSensorPin);
  
  //Serial.print("pulse duration:");
  //Serial.print(pulseDuration);Serial.print(" ");Serial.print(reading);Serial.print(" ");
  
  //Serial.print("time for full rev. (microsec.):");
  //Serial.println(pulseDuration*2);
  Serial.print("FREQ:");Serial.print(frequency/2);Serial.print("Hz");Serial.print("  RPM:");Serial.println((frequency/2)*60);

}

long getFrequency(int pin) {
  #define SAMPLES 4096
  long freq = 0;
  for(unsigned int j=0; j<SAMPLES; j++) freq+= 500000/pulseIn(pin, HIGH, 250000);
  return freq / SAMPLES;
}


void readFanRPMSensor(int speedSensorPin){
  //fanRevolutions = 0;   
  delay(1000);
  //fanRevolutions = fanRevolutions * 30;
  Serial.print ("     ");
  //Serial.print (fanRevolutions, DEC);
  Serial.print (" rpm");

   
  
}


void computeSafeAverageForArray(unsigned long arrayOfLongs){

  unsigned long cumulativeAverage = 0;
  unsigned long cumulativeRemainder = 0;
  unsigned long addendum = 0;
  unsigned long numberOfValues = 0;
  for (unsigned long currentValue = 0; currentValue != 100; ++currentValue) {
    ++numberOfValues;
    addendum = currentValue - cumulativeAverage + cumulativeRemainder;
    cumulativeAverage += addendum / numberOfValues;
    cumulativeRemainder = addendum % numberOfValues;
  }
  //return cumulativeAverage;
  
}




















