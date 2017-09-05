/*
 * Serial_Coder.h
 *
 *  Created on: Aug 28, 2017
 *      Author: steven
 *
 *
 * This file is used for encoding, decoding, sending, and receiving messages that will be sent over Arduino's serial communication.
 * The purpose is to make this sending more robust by implementing features like start and end characters to make sure messages are sent and received in whole.
 * Furthermore it will dedicate a byte to indicate the type of message, such that multiple different messages can be sent depending on the indicator byte.
 * A message will therefore look like {<start character> <message type> <message payload> <end character>}
 */

#include <stdio.h>
#include <stdlib.h>


#ifndef SERIAL_CODER_H
#define SERIAL_CODER_H


#define MAX_MESSAGE 10
#define IN_MESSAGE_SIZE 4
#define OUT_MESSAGE_SIZE 7
#define END_MARKER 255
#define SPECIAL_BYTE 253
#define START_MARKER 254
#define IN_MESSAGES 3
#define OUT_MESSAGES 1

// Message in types
#define VX 0
#define VY 1
#define Z 2

// Message out types
#define R 0




extern void Serial_Coder_Init();
extern void sendFloat(uint8_t msgtype, float outfloat);
extern float receiveFloat(uint8_t msgtype);
extern void getSerialData();

/*
extern void getSerialData();
extern void processData();
extern void decodeHighBytes();





extern void encodeHighBytes(uint8_t* sendData, uint8_t msgSize);

extern void checkBigEndian();

*/

/*
extern uint8_t _bytesRecvd;
extern uint8_t _dataSentNum;
extern uint8_t _dataRecvCount;

//extern uint8_t _dataRecvd[MAX_MESSAGE];
extern uint8_t _dataSend[MAX_MESSAGE];
extern uint8_t _tempBuffer[MAX_MESSAGE];

extern uint8_t _dataSendCount;
extern uint8_t _dataTotalSend;

extern bool _inProgress;
extern bool _startFound;
extern bool _allReceived;

extern MessageIn _receiveMessages[IN_MESSAGES];
extern MessageOut _sendMessages[OUT_MESSAGES];

extern uint8_t _varByte;

extern bool _bigEndian;
*/


#endif /* SERIAL_CODER_H */
