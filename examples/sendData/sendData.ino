#include <SoftwareSerial.h>
#include <RadioBlock.h>

//Pins connected to RadioBlock pins 1/2/3/4
RadioBlockSerialInterface interface = RadioBlockSerialInterface(5,4,3,2);

uint8_t payload[] = {2};

#define CODE_TEMP   1
#define CODE_ALARM  2
#define CODE_LED    3
#define CODE_VALVE  4

#define OUR_ADDRESS   0x1000
#define THEIR_ADDRESS 0x1001

void setup()  
{
  interface.begin();  

  //Give RadioBlock time to init
  delay(500);

  //Tell the world we are alive  
  interface.setLED(true);
  delay(1000);
  interface.setLED(false);
  
  interface.setChannel(15);
  interface.setPanID(0xBAAD);
  interface.setAddress(OUR_ADDRESS);
    
  Serial.begin(9600); 
  Serial.println("Starting...");
}

void loop() // run over and over
{
  //New Message
  if (interface.getResponse().isAvailable()) {
    Serial.println("New Response...");
  }
  
  //Oops?
  if (interface.getResponse().isError()) {
    Serial.println("You had ONE job Arduino. ONE job.");
  }
  
  delay(2000);
  
  //We use the 'setupMessage()' call if we want to use a bunch of data,
  //otherwise can use sendData() calls to directly send a few bytes
  
  //This is the OTHER guys address
  interface.setupMessage(THEIR_ADDRESS);
  
  //Send temperature reading
  interface.addData(CODE_TEMP, analogRead(A0));
  
  //Send state of pot (potentimeter, not drug manufacturing)
  interface.addData(CODE_VALVE, analogRead(1));
  
  //Toggle other other guys LED on RadioBlock
  interface.addData(CODE_LED, 1);
 
  //Send data OTA
  interface.sendMessage();  
}
