/*
 * Serial_Communication.h
 *
 *  Created on: Jul 25, 2017
 *      Author: steven
 */

#ifndef SW_AIRBORNE_MODULES_DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_
#define SW_AIRBORNE_MODULES_DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_

#include <stdint.h>
#include <stdbool.h>


// Some meta data for serial communication
#define MAX_MESSAGE 10
#define IN_MESSAGE_SIZE 4
#define OUT_MESSAGE_SIZE 7
#define END_MARKER 255
#define SPECIAL_BYTE 253
#define START_MARKER 254
#define NODE_STATE_SIZE 4

// Size of a floating point number
#define FLOAT_SIZE 4

// Setting up how many nodes can maximally be in the network
#define MAX_NODES 5

// How many nodes actually are in the network
#define NUM_NODES 3

// How many distant nodes are in the network (one less than the toal number of nodes)
#define DIST_NUM_NODES NUM_NODES-1

// Serial message types
#define VX 0
#define VY 1
#define Z 2
#define R 3


extern void decawave_serial_init(void);
extern void decawave_serial_periodic(void);
extern void decawave_serial_event(void);

extern void getSerialData(void);
extern void sendFloat(uint8_t msgtype, float outfloat);





#endif /* SW_AIRBORNE_MODULES_DECAWAVE_SERIAL_SERIAL_COMMUNICATION_H_ */
