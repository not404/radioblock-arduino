/**
 * Copyright (c) 2012 Colorado Micro Devices. All rights reserved.
 *
 * This file is part of RadioBlock-Arduino.
 *
 * Based on the XBee-Arduino file by Andrew Rapp
 *
 * RadioBlock-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RadioBlock-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RadioBlock-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RadioBlock.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "SoftwareSerial.h"

RadioBlockResponse::RadioBlockResponse() {

}

uint16_t RadioBlockResponse::getCrc() {
	return _crc;
}

void RadioBlockResponse::setCrc(uint16_t crc) {
	_crc = crc;
}

uint8_t RadioBlockResponse::getFrameDataLength() {
	return _frameLength;
}

void RadioBlockResponse::setFrameLength(uint8_t frameLength) {
	_frameLength = frameLength;
}

bool RadioBlockResponse::isAvailable() {
	return _complete;
}

void RadioBlockResponse::setAvailable(bool complete) {
	_complete = complete;
}

bool RadioBlockResponse::isError() {
	return _errorCode > 0;
}

RadioBlock_Status_t RadioBlockResponse::getErrorCode() {
	return _errorCode;
}

void RadioBlockResponse::setErrorCode(RadioBlock_Status_t errorCode) {
	_errorCode = errorCode;
}

void RadioBlockResponse::setCommandId(RadioBlock_CommandId_t commandId) {
	_commandId = commandId;
}

RadioBlock_CommandId_t RadioBlockResponse::getCommandId(void) {
	return _commandId;
}

void RadioBlockResponse::setPacketLength(uint8_t len){
	_length = len;
}

uint8_t RadioBlockResponse::getPacketLength() {
	return _length;
}

uint8_t* RadioBlockResponse::getFrameData() {
	return _frameDataPtr;
}

void RadioBlockResponse::setFrameData(uint8_t* frameDataPtr) {
	_frameDataPtr = frameDataPtr;
}

void RadioBlockResponse::init() {
	_complete = false;
	_errorCode = APP_STATUS_SUCESS;
	_crc = 0;
}

void RadioBlockResponse::reset() {
	init();
	_commandId = APP_COMMAND_ACK;
	_crc = 0;

	_errorCode = APP_STATUS_SUCESS;

	for (int i = 0; i < MAX_FRAME_DATA_SIZE; i++) {
		getFrameData()[i] = 0;
	}
}

RadioBlockCrc::RadioBlockCrc(){
	_crc = 0x1234;
}

void RadioBlockCrc::reset(){
	_crc = 0x1234;
}

void RadioBlockCrc::addByte(uint8_t data){
	data ^= _crc & 0xff;
	data ^= data << 4;
	_crc = (((uint16_t)data << 8) | ((_crc >> 8) & 0xff)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3);
}

uint16_t RadioBlockCrc::getCrc(void){
	return _crc;
}

RadioBlockSerialInterface::RadioBlockSerialInterface(int pin1, int pin2, uint8_t pin3, uint8_t pin4): _serial_internal(SoftwareSerial(pin4, pin3)) {
	//Pin1 = VCC
	//Pin2 = GND
	//Pin3 = Transmit from AVR
	//Pin4 = Receive to AVRs
	
	// default
	_baud = 115200;
	
	if (pin2 > 0) {
		pinMode(pin2, OUTPUT);
		digitalWrite(pin2, LOW);		
	}
	
	if (pin1 > 0) {
		pinMode(pin1, OUTPUT);
		digitalWrite(pin1, HIGH);
	}
	
	_serial = &_serial_internal;
	
	_pin1 = pin1;
	_pin2 = pin2;
	_pin3 = pin3;
	_pin4 = pin4;	
	
}


void RadioBlockSerialInterface::setBaud(long baud){
	_baud = baud;
}


void RadioBlockSerialInterface::setSerial(SoftwareSerial &serial) {
	_serial = &serial;
}

void RadioBlockSerialInterface::begin() {
	_serial->begin(_baud);
}

bool RadioBlockSerialInterface::available() {
	return _serial->available();
}

uint8_t RadioBlockSerialInterface::read() {
	return _serial->read();
} 

void RadioBlockSerialInterface::flush() {
	_serial->flush();
} 

void RadioBlockSerialInterface::write(uint8_t val) {
	_serial->write(val);
}

RadioBlockResponse& RadioBlock::getResponse() {
	return _response;
}

RadioBlock::RadioBlock(){
	_pos = 0;
	_response.init();
	_response.setFrameData(_responseFrameData);
}

void RadioBlock::SendReq_uint8(RadioBlock_CommandId_t command, uint8_t data) {
	resetResponse();
	_request.setCommandId(command);
	_request.setPayload(_smallTxFrame);
	_request.setPayloadLength(1);
	_smallTxFrame[0] = data;
	send(_request);
}

void RadioBlock::SendReq_uint16(RadioBlock_CommandId_t command, uint16_t data) {
	resetResponse();
	_request.setCommandId(command);
	_request.setPayload(_smallTxFrame);
	_request.setPayloadLength(2);
	_smallTxFrame[2]= ( data >> 8) & 0xff;
	_smallTxFrame[3]= ( data >> 0) & 0xff;
	send(_request);
}

void RadioBlock::SendReq_uint32(RadioBlock_CommandId_t command, uint32_t data) {
	resetResponse();
	_request.setCommandId(command);
	_request.setPayload(_smallTxFrame);
	_request.setPayloadLength(4);
	_smallTxFrame[0]= ( data >> 24) & 0xff;
	_smallTxFrame[1]= ( data >> 16) & 0xff;
	_smallTxFrame[2]= ( data >> 8) & 0xff;
	_smallTxFrame[3]= ( data >> 0) & 0xff;
	send(_request);
}

void RadioBlock::setLED(bool status) {
	uint8_t led = 0;
	if (status) {
		led = 1;
	}	
	SendReq_uint8(APP_COMMAND_SET_LED_STATE_REQ, led);
}

void RadioBlock::toggleLED(void) {
	SendReq_uint8(APP_COMMAND_SET_LED_STATE_REQ, 2);
}

void RadioBlock::sleepRequest(uint32_t sleepTimeMilliseconds) {
	SendReq_uint32(APP_COMMAND_SLEEP_REQ, sleepTimeMilliseconds);
}

void RadioBlock::sendData(unsigned int dest, unsigned char data) {
	resetResponse();
	_request.setCommandId(APP_COMMAND_DATA_REQ);
	_request.setPayload(_smallTxFrame);
	_request.setPayloadLength(5);
	_smallTxFrame[0]= dest & 0xff;
	_smallTxFrame[1]= (dest >> 8) & 0xff;
	_smallTxFrame[2]= 0x01; //Ack
	_smallTxFrame[3]= 0x00; //Handle
	_smallTxFrame[4] = data;
	send(_request);	
}
	
bool RadioBlock::available() {
	return false;
}

uint8_t RadioBlock::read() {
	return 0;
}

void RadioBlock::write(uint8_t val) {

}

void RadioBlock::flush(void) {

}

void RadioBlock::getResponse(RadioBlockResponse &response) {
	response.setFrameLength(_response.getFrameDataLength());
	response.setFrameData(_response.getFrameData());
}

void RadioBlock::readPacketUntilAvailable() {
	while (!(getResponse().isAvailable() || getResponse().isError())) {
		// read some more
		readPacket();
	}
}

bool RadioBlock::readPacket(int timeout) {

	if (timeout < 0) {
		return false;
	}

	unsigned long start = millis();

    while (int((millis() - start)) < timeout) {

     	readPacket();

     	if (getResponse().isAvailable()) {
     		return true;
     	} else if (getResponse().isError()) {
     		return false;
     	}
    }

    // timed out
    return false;
}

void RadioBlock::resetResponse() {	
	_response.reset();
}

void RadioBlock::readPacket() {
	// reset previous response
	if (_response.isAvailable() || _response.isError()) {
		// discard previous packet and start over
		resetResponse();
	}

    while (available()) {

        b = read();

        switch(_pos) {
			case 0:
		        if (b == START_BYTE) {
		        	_pos++;
		        }

		        break;
			case 1:
				// length 
				_response.setPacketLength(b);
				_pos++;
				break;

			case 2:
				_response.setCommandId((RadioBlock_CommandId_t)b);
				_pos++;				
				_crc.addByte(b);				
				break;
				
			default:
				// starts at third byte

				if (_pos > MAX_FRAME_DATA_SIZE) {
					// exceed max size.  should never occur
					_response.setErrorCode(APP_STATUS_INVALID_SIZE);
					return;
				}

				
				//PacketLength only includes commandid+payload, need to compensate for length+
				//start byte, then subtract one due to fact pos hasn't been incremented yet
				if (_pos == (_response.getPacketLength() + 3 - 1)) {					
					_response.setCrc(b);
					_pos++;
				} else if (_pos == (_response.getPacketLength() + 4 - 1)) {					
					_response.setCrc(_response.getCrc() | ((uint16_t)b << 8));
									
					if (_crc.getCrc() == _response.getCrc()) {						
						_response.setAvailable(true);
						_response.setErrorCode(APP_STATUS_SUCESS);
					} else {
						// checksum failed						
						_response.setAvailable(true);
						_response.setErrorCode(APP_STATUS_INVALID_CRC);
						_response.setCrc(_crc.getCrc());
					}

					// minus 4 because we start after start,msb,lsb,api and up to but not including checksum
					// e.g. if frame was one byte, _pos=4 would be the byte, pos=5 is the checksum, where end stop reading
					_response.setFrameLength(_pos - 4);

					// reset state vars
					_pos = 0;
					_crc.reset();

					return;
				} else {
					// add to packet array, starting with the third byte
					_response.getFrameData()[_pos - 3] = b;
					_pos++;
					
					_crc.addByte(b);
					
				}
        }
    }
}

void RadioBlock::send(RadioBlockRequest &request) {

	write(START_BYTE);

	// send length including command ID
	write(request.getPayloadLength() + 1);

	//Starting CRC of 0x1234
	_txcrc.reset();
	
	//Send command ID
	uint8_t data = (uint8_t)request.getCommandId();
   
	for (int i = 0; i <= request.getPayloadLength(); i++) {				
		write(data);
		
		//Perform CRC calculation while sending
		_txcrc.addByte(data);
		
		data = *(request.getPayloadData() + i);
	}

	// send CRC we calculated
	write(_txcrc.getCrc() & 0xff);
	write((_txcrc.getCrc() >> 8) & 0xff);

	// make sure packet is actually sent
	flush();
}


RadioBlockRequest::RadioBlockRequest(RadioBlock_CommandId_t commandId, uint8_t *payload, uint8_t payloadLength){
	_commandId = commandId;
	_payloadPtr = payload;
	_payloadLength = payloadLength;
}

RadioBlockRequest::RadioBlockRequest(){
	_commandId = APP_COMMAND_ACK;
	_payloadPtr = NULL;
	_payloadLength = 0;
}

uint8_t* RadioBlockRequest::getPayloadData() {
	return _payloadPtr;
}

void RadioBlockRequest::setPayload(uint8_t* payload) {
	_payloadPtr = payload;
}

uint8_t RadioBlockRequest::getPayloadLength() {
	return _payloadLength;
}

void RadioBlockRequest::setPayloadLength(uint8_t payloadLength) {
	_payloadLength = payloadLength;
}

void RadioBlockRequest::setCommandId(RadioBlock_CommandId_t commandId) {
	_commandId = commandId;
}

RadioBlock_CommandId_t RadioBlockRequest::getCommandId() {
	return _commandId;
}

