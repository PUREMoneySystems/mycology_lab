
/* Welcome to the ECU Reader project. This sketch uses the Canbus library.
  Originally created by SK Pang Electronics www.skpang.co.uk
  Modified to work with Jaycon Systems CAN Bus Shield
*/

#include <Canbus.h>
char buffer[512];

void setup() {
  Serial.begin(9600);
  Serial.println("ECU Reader");  /* For debug use */
  delay(1000);
  if(Canbus.init(CANSPEED_500))  /* Initialise MCP2515 CAN controller at the specified speed */
  {
    Serial.print("CAN Init ok");
  }else
  {
    Serial.print("Can't init CAN");
  }  
  delay(1000);
}

void loop() {
  if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1)          // Request for engine RPM */
  {
    Serial.println(buffer);                           // Print the RPM to the Serial Port                       
  } 
  delay(100); 
  if(Canbus.ecu_req(VEHICLE_SPEED,buffer) == 1)      // Request for engine Vehicle Speed */
  {
    Serial.println(buffer);
  }
  delay(100); 
  if(Canbus.ecu_req(ENGINE_COOLANT_TEMP,buffer) == 1)  // Request for engine Coolent Temperature*/
  {
    Serial.println(buffer);
  }
  delay(100); 
  if(Canbus.ecu_req(THROTTLE,buffer) == 1)      // Request for engine Throttle Position */
  {
    Serial.println(buffer);
  }  
   delay(100); 
}
