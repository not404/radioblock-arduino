#include <SoftwareSerial.h>
#include <RadioBlock.h>

//Pins connected to RadioBlock pins 1/2/3/4
RadioBlockSerialInterface interface = RadioBlockSerialInterface(5,4,3,2);

uint8_t payload[] = {2};

void setup()  
{
  interface.begin();  
  Serial.begin(9600); 
  Serial.println("Starting...");
}

void loop() // run over and over
{
  interface.setLED(true);
  delay(500);
  interface.setLED(false);
  delay(1500);
  interface.toggleLED();
  
  if (interface.readPacket(1000)) {
    if (interface.getResponse().getErrorCode() == APP_STATUS_SUCESS){
      Serial.println("Party");
    } else {
     Serial.println("I failed");
    }
     Serial.print("Len: ");
     Serial.print(interface.getResponse().getPacketLength(), DEC);
     Serial.print(", Command: ");
     Serial.print(interface.getResponse().getCommandId(), HEX);
     Serial.print(", CRC: ");
     Serial.print(interface.getResponse().getCrc(), HEX);
     Serial.println("");
  }

  
}
