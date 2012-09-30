/**
 * Copyright (c) 2012 Colorado Micro Devices. All rights reserved.
 *
 * Based on the XBee-Arduino file by Andrew Rapp
 *
 * This file is part of RadioBlock-Arduino.
 */

#ifndef RadioBlock_h
#define RadioBlock_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
	#include "SoftwareSerial.h"
#else
	#include "WProgram.h"
#endif

#include <inttypes.h>

#define START_BYTE 0xAB

/* Larger frames possible - but unlikely, so to save SRAM we set this to largest reasonable size */
#define MAX_FRAME_DATA_SIZE 128 

#define BROADCAST_ADDRESS 0xffff

typedef enum RadioBlock_Status_t
{
  APP_STATUS_SUCESS               = 0x00,
  APP_STATUS_INVALID_SIZE         = 0x80,
  APP_STATUS_INVALID_CRC          = 0x81,
  APP_STATUS_TIMEOUT              = 0x82,
  APP_STATUS_UNKNOWN_COMMAND      = 0x83,
  APP_STATUS_MALFORMED_COMMAND    = 0x84,
  APP_STATUS_FLASH_ERROR          = 0x85,
  APP_STATUS_INVALID_PAYLOAD_SIZE = 0x86,
} RadioBlock_Status_t;

typedef enum RadioBlock_CommandId_t
{
  APP_COMMAND_ACK                   = 0x00,

  APP_COMMAND_TEST_REQ              = 0x01,
  APP_COMMAND_TEST_RESP             = 0x02,

  APP_COMMAND_RESET_REQ             = 0x03,
  APP_COMMAND_SETTINGS_REQ          = 0x04,
  APP_COMMAND_SET_UART_MODE_REQ     = 0x05,
  APP_COMMAND_SLEEP_REQ             = 0x06,
  APP_COMMAND_WAKEUP_IND            = 0x07,

  APP_COMMAND_DATA_REQ              = 0x20,
  APP_COMMAND_DATA_CONF             = 0x21,
  APP_COMMAND_DATA_IND              = 0x22,

  APP_COMMAND_SET_ADDR_REQ          = 0x23,
  APP_COMMAND_GET_ADDR_REQ          = 0x24,
  APP_COMMAND_GET_ADDR_RESP         = 0x25,

  APP_COMMAND_SET_PANID_REQ         = 0x26,
  APP_COMMAND_GET_PANID_REQ         = 0x27,
  APP_COMMAND_GET_PANID_RESP        = 0x28,

  APP_COMMAND_SET_CHANNEL_REQ       = 0x29,
  APP_COMMAND_GET_CHANNEL_REQ       = 0x2a,
  APP_COMMAND_GET_CHANNEL_RESP      = 0x2b,

  APP_COMMAND_SET_RX_STATE_REQ      = 0x2c,
  APP_COMMAND_GET_RX_STATE_REQ      = 0x2d,
  APP_COMMAND_GET_RX_STATE_RESP     = 0x2e,

  APP_COMMAND_SET_TX_POWER_REQ      = 0x2f,
  APP_COMMAND_GET_TX_POWER_REQ      = 0x30,
  APP_COMMAND_GET_TX_POWER_RESP     = 0x31,

  APP_COMMAND_SET_SECURITY_KEY_REQ  = 0x32,

  APP_COMMAND_SET_ACK_STATE_REQ     = 0x35,
  APP_COMMAND_GET_ACK_STATE_REQ     = 0x36,
  APP_COMMAND_GET_ACK_STATE_RESP    = 0x37,

  APP_COMMAND_SET_PER_MODE_REQ      = 0x40,

  APP_COMMAND_SET_LED_STATE_REQ     = 0x80,
} RadioBlock_CommandId_t;

typedef enum RadioBlock_PerMode_t
{
  APP_PERMODE_NONE                  = 0x00,
  APP_PERMODE_TX_ONE                = 0x01,
  APP_PERMODE_TX_CONT               = 0x02,
  APP_PERMODE_RX                    = 0x03,
  APP_PERMODE_REPORTER              = 0x04,
} RadioBlock_PerMode_t;

enum
{
  APP_SETTINGS_SAVE                 = 0x10,
  APP_SETTINGS_SET_DEFAULTS         = 0x15,
};

class RadioBlockCrc {
public:
	RadioBlockCrc();
	void reset();
	void addByte(uint8_t data);
	uint16_t getCrc();
private:
	uint16_t _crc;
};


class RadioBlockResponse {
public:
	/**
	 * Default constructor
	 */
	RadioBlockResponse();
	/**
	 * Returns Command Id of the response
	 */
	RadioBlock_CommandId_t getCommandId();
	void setCommandId(RadioBlock_CommandId_t commandId);
	/**
	 * Returns the length of the packet
	 */
	uint8_t getPacketLength();
	void setPacketLength(uint8_t length);	
	/**
	 * Returns the packet CRC-16
	 */
	uint16_t getCrc();
	void setCrc(uint16_t crc);
	/**
	 * Returns the length of the frame data: Not including CRC-16, start byte, or length
	 */
	uint8_t getFrameDataLength();
	void setFrameData(uint8_t* frameDataPtr);

	uint8_t* getFrameData();

	void setFrameLength(uint8_t frameLength);

	/**
	 * Resets the response to default values
	 */
	void reset();
	/**
	 * Initializes the response
	 */
	void init();

	/**
	 * Returns true if the response has been successfully parsed and is complete and ready for use
	 */
	bool isAvailable();
	void setAvailable(bool complete);
	/**
	 * Returns true if the response contains errors
	 */
	bool isError();
	/**
	 * Returns an error code, or zero, if successful.
	 */
	RadioBlock_Status_t getErrorCode();
	void setErrorCode(RadioBlock_Status_t errorCode);
protected:
	// pointer to frameData
	uint8_t* _frameDataPtr;
private:
	RadioBlock_CommandId_t _commandId;
	uint8_t _length;
	uint16_t _crc;
	uint8_t _frameLength;
	bool _complete;
	RadioBlock_Status_t _errorCode;
};

class RadioBlockRequest {
public:
	RadioBlockRequest();

	RadioBlockRequest(RadioBlock_CommandId_t commandId, uint8_t *payload, uint8_t payloadLength);
	/**
	 * Sets the command id. 
	 */
	void setCommandId(RadioBlock_CommandId_t commandId);
	/**
	 * Returns the frame id
	 */
	RadioBlock_CommandId_t getCommandId();
	/**
	 * Returns the payload of the packet, if not null
	 */
	uint8_t* getPayloadData();
	/**
	 * Sets the payload array
	 */
	void setPayload(uint8_t* payloadPtr);
	/**
	 * Returns the length of the payload array, as specified by the user.
	 */
	uint8_t getPayloadLength();
	/**
	 * Sets the length of the payload to include in the request.  For example if the payload array
	 * is 50 bytes and you only want the first 10 to be included in the packet, set the length to 10.
	 * Length must be <= to the array length.
	 */
	void setPayloadLength(uint8_t payloadLength);	
private:
	RadioBlock_CommandId_t _commandId;
	uint8_t* _payloadPtr;
	uint8_t _payloadLength;
};


class RadioBlock {
public:

	RadioBlock();
	
	void setLED(bool status);
	void sendData(unsigned int dest, unsigned char data);
	void toggleLED();
	void sleepRequest(uint32_t sleepTimeMilliseconds);
	//RadioBlock_Status_t sendData(uint16_t dest, uint8_t * dataPtr, uint8_t dataLen);
	//receiveDataPoll();
	//receiveDataCallback();
	//setAddress(uint16_t address);
	//uint16_t getAddress();
	//setPanID(uint16_t panid);
	//uint16_t getPanID(void);
	//setChannel(uint8_t channel);
	//uint8_t getChannel();
	
	void setupMessage(uint16_t dest);
	void addData(uint8_t code, uint8_t data);
	void addData(uint8_t code, int8_t data);
	void addData(uint8_t code, uint16_t data);
	void addData(uint8_t code, int16_t data);
	void addData(uint8_t code, uint32_t data);
	void addData(uint8_t code, int32_t data);
	//void addData(uint8_t code, float data);
	void sendMessage(void);
	
	/**
	 * Returns a reference to the current response
	 * Note: once readPacket is called again this response will be overwritten!
	 */
	RadioBlockResponse& getResponse();
	
	/**
	 * Sends a RadioBlockRequest (TX packet) out the serial port
	 */
	void send(RadioBlockRequest &request);
	
	void readPacket();
	/**
	 * Waits a maximum of <i>timeout</i> milliseconds for a response packet before timing out; returns true if packet is read.
	 * Returns false if timeout or error occurs.
	 */
	bool readPacket(int timeout);
	/**
	 * Reads until a packet is received or an error occurs.
	 * Caution: use this carefully since if you don't get a response, your Arduino code will hang on this
	 * call forever!! often it's better to use a timeout: readPacket(int)
	 */
	void readPacketUntilAvailable();
	
	/**
	 * Check if the RadioLego is present
	 */
	void begin();
	
	void getResponse(RadioBlockResponse &response);

protected:
	void resetResponse();
	RadioBlockRequest  _request;
	RadioBlockResponse _response;
	uint8_t _txFrameBuffer[MAX_FRAME_DATA_SIZE];
	uint8_t _responseFrameData[MAX_FRAME_DATA_SIZE];
	uint8_t _pos;
	RadioBlockCrc _txcrc;
	RadioBlockCrc _crc;
	uint8_t b;
	
	//Could just use overloaded SendReq(), but explicitly saying data type
	//makes less likely to accidently call wrong function when copy/pasting
	//versions of code
	void SendReq_uint32(RadioBlock_CommandId_t command, uint32_t data);
	void SendReq_uint16(RadioBlock_CommandId_t command, uint16_t data);
	void SendReq_uint8(RadioBlock_CommandId_t command, uint8_t data);
	
	virtual bool available();
	virtual uint8_t read();
	virtual void write(uint8_t val);
	virtual void flush(void);

};

class RadioBlockSerialInterface : public RadioBlock {
public:
	/**
	 * When creating the RadioBlock, map pin1/2/3/4 on the RadioBlock to the proper pin on the Arduino.
	 * Pin1=VCC, Pin2=GND, Pin3=TX from AVR, Pin4=RX to AVR.
	 * You may specify pin1/pin2 as '-1' if you are providing power in normal ways. You may specify
	 * pin1/pin2 as arduino pins when you have plugged the RadioBlock directly into the Arduino.
	 */
	RadioBlockSerialInterface(int pin1, int pin2, uint8_t pin3, uint8_t pin4);	
	
	/**
	 * Open serial port, find RadioLego
	 */
	void begin();
		
	/**
	 * Set the baud rate of the serial interface
	 */
	void setBaud(long baud);
	
	/**
	 * Specify the serial port. Note that this is automatically setup in the call to setupPins,
	 * so you SHOULD NOT need to specify this.
	 */
	void setSerial(SoftwareSerial &serial);	
	
private:		
	int _pin1, _pin2;
	uint8_t _pin3, _pin4;
	SoftwareSerial* _serial;
	SoftwareSerial _serial_internal;
	long _baud;
	bool available();
	uint8_t read();
	void write(uint8_t val);
	void flush(void);
};

/*
class RadioBlockOTAInterface {

};
*/

#endif

