/*
 * Serial_Coder.c
 *
 *  Created on: Aug 28, 2017
 *      Author: steven
 */

#include "Serial_Coder.h"

struct MessageIn{
	uint8_t type;
	uint8_t msg[IN_MESSAGE_SIZE];
};

struct MessageOut{
	uint8_t type;
	uint8_t msg[OUT_MESSAGE_SIZE];
};


static uint8_t _bytesRecvd = 0;
static uint8_t _dataSentNum = 0;
static uint8_t _dataRecvCount = 0;

//static uint8_t _dataRecvd[MAX_MESSAGE];
static uint8_t _dataSend[MAX_MESSAGE];
static uint8_t _tempBuffer[MAX_MESSAGE];

static uint8_t _dataSendCount = 0;
static uint8_t _dataTotalSend = 0;

static bool _inProgress = false;
static bool _startFound = false;
static bool _allReceived = false;

static struct MessageIn _receiveMessages[IN_MESSAGES];
//static struct MessageOut _sendMessages[OUT_MESSAGES];

static uint8_t _varByte = 0;

static bool _bigEndian = false;


static void decodeHighBytes();
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize);
static void checkBigEndian(void);
static void getSerialData();




static void Serial_Coder_init(){

}




/**
 * Function for receiving serial data.
 * Only receives serial data that is between the start and end markers. Discards all other data.
 * Stores the received data in _tempBuffer, and after decodes the high bytes and copies the final
 * message to the corresponding message in _messages.
 */
void getSerialData(){
	if (Serial.available()>0){
		_varByte = Serial.read();
		if (_varByte == START_MARKER){
			_bytesRecvd = 0;
			_inProgress = true;
		}

		if (_inProgress){
			_tempBuffer[_bytesRecvd] = _varByte;
			_bytesRecvd++;
		}

		if (_varByte == END_MARKER){
			_inProgress = false;
			_allReceived = true;

			decodeHighBytes();
		}
	}

}

/**
 * Function for decoding the high bytes of received serial data and saving the message.
 * Since the start and end marker could also be regular payload bytes (since they are simply the values
 * 254 and 255, which could also be payload data) the payload values 254 and 255 have been encoded
 * as byte pairs 253 1 and 253 2 respectively. Value 253 itself is encoded as 253 0.
 *  This function will decode these back into values the original payload values.
 */
static void decodeHighBytes(){
	_dataRecvCount = 0;
	uint8_t msgType = _tempBuffer[1];
	for (uint8_t i = 2; i<_bytesRecvd-1; i++){ // Skip the begin marker (0), message type (1), and end marker (_bytesRecvd-1)
		_varByte = _tempBuffer[i];
		if (_varByte == SPECIAL_BYTE){
			i++;
			_varByte = _varByte + _tempBuffer[i];
		}
		_receiveMessages[msgType].msg[_dataRecvCount] = _varByte;
		_dataRecvCount++;
	}
}

/**
 * Function used to receive a float with a certain message ID
 */
float receiveFloat(uint8_t msgtype){
	float tempfloat;
	memcpy(&tempfloat,&_receiveMessages[msgtype].msg,4);
	return tempfloat;
}

/**
 * Function that will send a float over serial. The actual message that will be sent will have
 * a start marker, the message type, 4 bytes for the float, and the end marker.
 */
void sendFloat(uint8_t msgtype, float outfloat){
	uint8_t floatbyte[4];
	memcpy(floatbyte,&outfloat,4);
	encodeHighBytes(floatbyte,4);
	Serial.write(START_MARKER);
	Serial.write(msgtype);
	Serial.write(_tempBuffer,_dataTotalSend);
	Serial.write(END_MARKER);

}

/**
 * Function that encodes the high bytes of the serial data to be sent.
 * Start and end markers are reserved values 254 and 255. In order to be able to send these values,
 * the payload values 253, 254, and 255 are encoded as 2 bytes, respectively 253 0, 253 1, and 253 2.
 */
static void encodeHighBytes(uint8_t* sendData, uint8_t msgSize){
	_dataSendCount = msgSize;
	_dataTotalSend = 0;
	for (uint8_t i = 0; i < _dataSendCount; i++){
		if (sendData[i] >= SPECIAL_BYTE){
			_tempBuffer[_dataTotalSend] = SPECIAL_BYTE;
			_dataTotalSend++;
			_tempBuffer[_dataTotalSend] = sendData[i] - SPECIAL_BYTE;
		}
		else{
			_tempBuffer[_dataTotalSend] = sendData[i];
		}
		_dataTotalSend++;
	}
}

/**
 * Function to check the endianness of the system
 */
static void checkBigEndian(void)
{
    union {
        uint32_t i;
        char c[4];
    } un = {0x01020304};

    _bigEndian = un.c[0] == 1;
}
