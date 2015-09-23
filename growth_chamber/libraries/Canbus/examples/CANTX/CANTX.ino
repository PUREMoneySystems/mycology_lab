/* Welcome to the CAN TX project. This sketch uses the Canbus library.
  Originally created by SK Pang Electronics www.skpang.co.uk
  Modified to work with Jaycon Systems CAN Bus Shield
  This is the CAN TX example, ment to work in conjunction with the CAN RX Example.
  
  You will need to have atleast two Arduino/CAN setup and connected over Ethernet cables.
  Make sure you have the termination jumpers soldered on the end nodes, and POE jumpered on both boards
  if you want to power only one side.
*/

#include <Canbus.h>


unsigned int MY_ID = 0x0756; // This is the devices ID. The Lower the Number, the Higher its Priority on the CAN Bus. ID 0x0000 would be the highest Priority. (Cant have two with the same ID)

//We wont use this for this example, but if you need to send messages back to the Master, we would need this
// unsigned char MY_PID = 1; // This is the ID we will use to check if the message was for this device.

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

unsigned char RX_Buff[8];

void setup() {
  Serial.begin(9600);
  Serial.println("CAN TX");  /* For debug use */
  delay(1000);
  
  if(Canbus.init(CANSPEED_500))  /* Initialise MCP2515 CAN controller at the specified speed */
    Serial.println("CAN Init ok");
  else
    Serial.println("Can't init CAN");
    
  delay(1000);
}

void loop() {
  
  //This is just a basic fading routine
  brightness = brightness + fadeAmount;
   
  if (brightness == 0 || brightness == 255) {
    fadeAmount = -fadeAmount ; 
  }  
  RX_Buff[0] = 10; // We want this message to be picked up by device with a PID of 10
  RX_Buff[1] = brightness; // This is the brightness level we want our LED set to on the other device
  
  Canbus.message_tx(MY_ID,RX_Buff); // Send the message on the CAN Bus to be picked up by the other devices
  delay(10); 
}
