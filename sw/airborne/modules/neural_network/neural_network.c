/*
 * Copyright (C) Thomas Fijen
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/neural_network/neural_network.c"
 * @author Thomas Fijen
 * This fuction implements the NN control necessary for my persistent surveillance mission. 
 * The NN used is not fully connected and has two outputs, namely; the x and y velocities.
 */

#include "modules/neural_network/neural_network.h"
#include "std.h"
#include "state.h"
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/navigation/waypoints.h"
#include "modules/decawave/uwb_localisation_and_comms.h"

/***** Neural Network Defines *****/

#ifndef MS_NUM_INPUTS
#define MS_NUM_INPUTS 18 
#endif

#ifndef MS_NUM_OUTPUTS
#define MS_NUM_OUTPUTS 2 // VX and VY 
#endif

#ifndef MS_LENGTH
#define MS_LENGTH 7.0f // Length of area (in m) (must be a multiple of grid_res)
#endif

#ifndef MS_BREADTH
#define MS_BREADTH 7.0f  // Breadth (width) of area (in m) (must be a multiple of grid_res)
#endif 

#ifndef MS_GRID_RES
#define MS_GRID_RES 0.5f // Distance between gridpoints along and x and y
#endif

#ifndef MS_NUM_NODES
#define MS_NUM_NODES 24 // Nodes within the network incl. input and output layer
#endif

#ifndef MS_NUM_CONNECT
#define MS_NUM_CONNECT 44 // Number of connections in the whole network
#endif

/*****************/

#ifndef MS_SENSOR_RANGE
#define MS_SENSOR_RANGE 4.0f // Radius from agent (360 degrees)
#endif

#ifndef MS_MAX_VEL
#define MS_MAX_VEL 1.0f 
#endif

#ifndef MS_SWARM_SIZE
#define MS_SWARM_SIZE 3
#endif

// TODO: Change to something more failsafe
#ifndef MS_CURRENT_ID
#define MS_CURRENT_ID MS_SWARM_SIZE-1
#endif

static struct MS_Struct msParams;
static struct NN_struct nnParams;

/** Mission Space Parameter structure */
struct MS_Struct {
    // uint8_t MS[(uint8_t) (MS_BREDTH/MS_GRID_RES)][(uint8_t) (MS_LENGTH/MS_GRID_RES)];
    uint8_t MS[20][20];
    float sensorRange;
	struct EnuCoor_f uavs[MS_SWARM_SIZE];
};

/** Structure containing NN parameters */
struct NN_struct {
    // At the moment the connections are input by hand here.
    // These are the weights of the origional connections
    float connectionsInit [2*MS_NUM_INPUTS];

    // Connections added by the NEAT
    // These are the weights of the added connections
    uint8_t connectFrom [MS_NUM_CONNECT-2*MS_NUM_INPUTS];
    
    // These are the weights of the added connections
    uint8_t connectTo [MS_NUM_CONNECT-2*MS_NUM_INPUTS];

    // These are the weights of the added connections
    float connectWeight [MS_NUM_CONNECT-2*MS_NUM_INPUTS];

    // Node Properties
    uint8_t outputIndex[MS_NUM_OUTPUTS];
    uint8_t node_ID [MS_NUM_NODES];
    float node_out [MS_NUM_NODES];
};

/** This function determines the inputs into the NN */
void calcInputs(){

    struct EnuCoor_f *pos = &(UAV);

    /* Conversion between coordinate systems to rectify cyberzoo (see picture on your phone, Mario) */
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

    struct EnuCoor_f *pos = stateGetPositionEnu_f();
    // float tempX=(*pos).x;
    // float tempY=(*pos).y;
    // (*pos).x = a*tempX+b*tempY+c;
    // (*pos).y = -b*tempX+a*tempY+d;

    for(uint8_t i=0;i<MS_SWARM_SIZE;i++){
        float temp[2];
        getPos_UWB(i,temp);
        msParams.uavs[i].x = temp[0];
        msParams.uavs[i].y = temp[1];
        // msParams.uavs[i].x = a*temp[0]+b*temp[1]+c;
        // msParams.uavs[i].y = b*temp[0]-a*temp[1]+d;
    }
    
    /* Default range values */
    for (uint8_t i = 0; i < MS_NUM_INPUTS; i++) {
        nnParams.node_out[i] = 0.0;
    }

    uint8_t currentCell_x;
    uint8_t currentCell_y;
    if ((*pos).x >= 0 && (*pos).x <= MS_LENGTH && (*pos).y >= 0 && (*pos).y <= MS_BREDTH) {
        /* Get current cell index */
        currentCell_x = (uint8_t) ((*pos).x/MS_GRID_RES);
        currentCell_y = (uint8_t) ((*pos).y/MS_GRID_RES);

        /* Antennae Function values */
        uint8_t numCells = msParams.sensorRange / MS_GRID_RES;

        nnParams.node_out[0] = MS_GRID_RES*(currentCell_y+1)-(*pos).y;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y+i*MS_GRID_RES) > MS_BREDTH) {
                break;
            }
            else if(msParams.MS[currentCell_y+i][currentCell_x] != 0) {
                nnParams.node_out[0] = nnParams.node_out[0] + MS_GRID_RES;
                nnParams.node_out[8] = (100-msParams.MS[currentCell_y+i][currentCell_x])/100.0;
                if (nnParams.node_out[0] >= msParams.sensorRange) {
                    nnParams.node_out[0] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[2] = MS_GRID_RES*(currentCell_x+1)-(*pos).x;
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).x+i*MS_GRID_RES) > MS_LENGTH) {
                break;
            }
            else if(msParams.MS[currentCell_y][currentCell_x+i] != 0) {
                nnParams.node_out[2] = nnParams.node_out[2] + MS_GRID_RES;
                nnParams.node_out[10] = (100-msParams.MS[currentCell_y][currentCell_x+i])/100.0;
                if (nnParams.node_out[2] >= msParams.sensorRange) {
                    nnParams.node_out[2] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[4] = (*pos).y - MS_GRID_RES*(currentCell_y);
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y-i][currentCell_x] != 0) {
                nnParams.node_out[4] = nnParams.node_out[4] + MS_GRID_RES;
                nnParams.node_out[12] = (100-msParams.MS[currentCell_y-i][currentCell_x])/100.0;
                if (nnParams.node_out[4] >= msParams.sensorRange) {
                    nnParams.node_out[4] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[6] = (*pos).x-MS_GRID_RES*(currentCell_x);
        for (uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).x-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y][currentCell_x-i] != 0) {
                nnParams.node_out[6] = nnParams.node_out[6] + MS_GRID_RES;
                nnParams.node_out[14] = (100-msParams.MS[currentCell_y][currentCell_x-i])/100.0;
                if (nnParams.node_out[6] >= msParams.sensorRange) {
                    nnParams.node_out[6] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        /* Diagonal antennae */
        float stepSize = MS_GRID_RES/cosf(M_PI/4);

        nnParams.node_out[1] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y+1))*((*pos).y-MS_GRID_RES*(currentCell_y+1)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y+i*MS_GRID_RES) > MS_BREDTH || ((*pos).x+i*MS_GRID_RES) > MS_LENGTH) {
                break;
            }
            else if(msParams.MS[currentCell_y+i][currentCell_x+i] != 0) {
                nnParams.node_out[1] = nnParams.node_out[1] + stepSize;
                nnParams.node_out[9] = (100-msParams.MS[currentCell_y+i][currentCell_x+i])/100.0;
                if (nnParams.node_out[1] >= msParams.sensorRange) {
                    nnParams.node_out[1] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        nnParams.node_out[3] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x+1))*((*pos).x-MS_GRID_RES*(currentCell_x+1))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y-i*MS_GRID_RES) < 0 || ((*pos).x+i*MS_GRID_RES) > MS_BREDTH) {
                break;
            }
            else if(msParams.MS[currentCell_y-i][currentCell_x+i] != 0) {
                nnParams.node_out[3] = nnParams.node_out[3] + stepSize;
                nnParams.node_out[11] = (100-msParams.MS[currentCell_y-i][currentCell_x+i])/100.0;
                if (nnParams.node_out[3] >= msParams.sensorRange) {
                    nnParams.node_out[3] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[5] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x))*((*pos).x-MS_GRID_RES*(currentCell_x))+((*pos).y-MS_GRID_RES*(currentCell_y))*((*pos).y-MS_GRID_RES*(currentCell_y)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y-i*MS_GRID_RES) < 0 || ((*pos).x-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y-i][currentCell_x-i] != 0) {
                nnParams.node_out[5] = nnParams.node_out[5] + stepSize;
                nnParams.node_out[13] = (100-msParams.MS[currentCell_y-i][currentCell_x-i])/100.0;
                if (nnParams.node_out[5] >= msParams.sensorRange) {
                    nnParams.node_out[5] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        nnParams.node_out[7] = sqrtf(((*pos).x-MS_GRID_RES*(currentCell_x))*((*pos).x-MS_GRID_RES*(currentCell_x))+((*pos).y-MS_GRID_RES*(currentCell_y+1))*((*pos).y-MS_GRID_RES*(currentCell_y+1)));
        for(uint8_t i = 1; i <= numCells; i++) {
            if (((*pos).y+i*MS_GRID_RES) > MS_BREDTH || ((*pos).x-i*MS_GRID_RES) < 0) {
                break;
            }
            else if(msParams.MS[currentCell_y+i][currentCell_x-i] != 0) {
                nnParams.node_out[7] = nnParams.node_out[7] + stepSize;
                nnParams.node_out[15] = (100-msParams.MS[currentCell_y+i][currentCell_x-i])/100.0;
                if (nnParams.node_out[7] >= msParams.sensorRange) {
                    nnParams.node_out[7] = msParams.sensorRange;
                    break;
                }
            }
            else{
                break;
            }
        }

        /* Correcting the range measurements to account for other UAVs */
        for(uint8_t i = 0; i < MS_SWARM_SIZE; i++){
            if (i != MS_CURRENT_ID) {
                if (msParams.uavs[i].x >= 0 && msParams.uavs[i].x <= MS_LENGTH && msParams.uavs[i].y >= 0 && msParams.uavs[i].y <= MS_BREDTH) {
                    uint8_t agentCell_x = (uint8_t) (msParams.uavs[i].x/MS_GRID_RES);
                    uint8_t agentCell_y = (uint8_t) (msParams.uavs[i].y/MS_GRID_RES);
                    if(agentCell_x == currentCell_x){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(agentCell_y >= currentCell_y && distance < nnParams.node_out[0]){
                            nnParams.node_out[0] = distance;
                        } else if (agentCell_y < currentCell_y && distance < nnParams.node_out[4]) {
                            nnParams.node_out[4] = distance;
                        }
                    }
                    else if (agentCell_y == currentCell_y){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(agentCell_x >= currentCell_x && distance < nnParams.node_out[2]){
                            nnParams.node_out[2] = distance;
                        } else if (agentCell_x < currentCell_x && distance < nnParams.node_out[6]) {
                            nnParams.node_out[6] = distance;
                        }
                    }
                    else if ((agentCell_y-currentCell_y) == (agentCell_x-currentCell_x) && agentCell_x >= currentCell_x ) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[1]){
                            nnParams.node_out[1] = distance;
                        }
                    } else if ((currentCell_y-agentCell_y) == (agentCell_x-currentCell_x) && agentCell_x >= currentCell_x){
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[3]){
                            nnParams.node_out[3] = distance;
                        }
                    } else if ((currentCell_y-agentCell_y) == (currentCell_x-agentCell_x) && agentCell_x < currentCell_x) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[5]){
                            nnParams.node_out[5] = distance;
                        }
                    } else if ((agentCell_y-currentCell_y) == (currentCell_x-agentCell_x) && agentCell_x < currentCell_x) {
                        float distance = sqrtf((msParams.uavs[i].x-(*pos).x)*(msParams.uavs[i].x-(*pos).x)+(msParams.uavs[i].y-(*pos).y)*(msParams.uavs[i].y-(*pos).y));
                        if(distance < nnParams.node_out[7]){
                            nnParams.node_out[7] = distance;
                        }
                    }
                }
            }

        }

        /* Boolean in area */
        nnParams.node_out[MS_NUM_INPUTS-2] = 1;
    }
    else{
        for (uint8_t i = 0; i < MS_NUM_INPUTS; i++){
            nnParams.node_out[i] = 0;
        }

        if ((*pos).x < 0) { 
            if ((*pos).y < 0) {
                nnParams.node_out[1] = msParams.sensorRange;
                nnParams.node_out[9] = 1.0;
            }else if ((*pos).y >= 0 && (*pos).y <= MS_BREDTH) {
                nnParams.node_out[2] = msParams.sensorRange;
                nnParams.node_out[10] = 1.0;
            }else {
                nnParams.node_out[3] = msParams.sensorRange;
                nnParams.node_out[11] = 1.0;
            }
        }else if ((*pos).x >=0 && (*pos).x <= MS_LENGTH){ 
            if ((*pos).y < 0) { 
                nnParams.node_out[0] = msParams.sensorRange;
                nnParams.node_out[8] = 1.0;
            }else if ((*pos).y > MS_BREDTH) { 
                nnParams.node_out[4] = msParams.sensorRange;
                nnParams.node_out[12] = 1.0;
            }
        }else { 
            if ((*pos).y < 0) { 
                nnParams.node_out[7] = msParams.sensorRange;
                nnParams.node_out[15] = 1.0;
            }else if ((*pos).y >= 0 && (*pos).y <= MS_BREDTH) { 
                nnParams.node_out[6] = msParams.sensorRange;
                nnParams.node_out[14] = 1.0;
            }else {
                nnParams.node_out[5] = msParams.sensorRange;
                nnParams.node_out[13] = 1.0;
            }
        }

        /* Boolean outside area */
        nnParams.node_out[MS_NUM_INPUTS-2] = 0;
    }

    /** Bias Node */
    nnParams.node_out[MS_NUM_INPUTS-1] = 1;

    /* Normalising the input values */
    nnParams.node_out[0] = nnParams.node_out[0]/msParams.sensorRange;
    nnParams.node_out[1] = nnParams.node_out[1]/msParams.sensorRange;
    nnParams.node_out[2] = nnParams.node_out[2]/msParams.sensorRange;
    nnParams.node_out[3] = nnParams.node_out[3]/msParams.sensorRange;
    nnParams.node_out[4] = nnParams.node_out[4]/msParams.sensorRange;
    nnParams.node_out[5] = nnParams.node_out[5]/msParams.sensorRange;
    nnParams.node_out[6] = nnParams.node_out[6]/msParams.sensorRange;
    nnParams.node_out[7] = nnParams.node_out[7]/msParams.sensorRange;
}

/** This function implements the activation function of the NN */
float activationFunction(float x) {
    float output = (expf(x)-expf(-x))/(expf(x)+expf(-x));

    return output;
}

/** This function calculates the outputs of the NN */
void calcNN() {
    uint8_t recurrentNN = 0;
    
    // Reset the node_out to zero for the new calculation
    for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        nnParams.node_out[nodeNum] = 0;
    }

    /* Determine the inputs to the NN */
    calcInputs();

    //Calculate the contributions of the initial connections
    float outNodeInput1;
    float outNodeInput2;

    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOutputs++){
        for (uint8_t numIn = 0; numIn < MS_NUM_INPUTS; numIn++){
            nnParams.node_out[nnParams.outputIndex[numOutputs]] = nnParams.node_out[nnParams.outputIndex[numOutputs]] + nnParams.connectionsInit[numIn+MS_NUM_INPUTS*numOutputs]*nnParams.node_out[numIn];
        }
    }
    outNodeInput1 = nnParams.node_out[nnParams.outputIndex[0]];
    outNodeInput2 = nnParams.node_out[nnParams.outputIndex[1]];

    // Calculate the contributions of the added connections and Nodes
    for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
        for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++ ) {
            if(nnParams.connectTo[connectNum] == nnParams.node_ID[nodeNum]) {
                float inValue = 0;
                for(uint8_t i = 0; i < MS_NUM_NODES; i++){
                    if(nnParams.connectFrom[connectNum] == nnParams.node_ID[i]) {
                        inValue = nnParams.node_out[i];
                    }
                }
                nnParams.node_out[nodeNum] = nnParams.node_out[nodeNum] + nnParams.connectWeight[connectNum]*inValue;
            }
        }
        nnParams.node_out[nodeNum] = activationFunction(nnParams.node_out[nodeNum]);
    }

    //Case when there are recurrent connections:
    if(recurrentNN == 1) {
        float no_change_threshold=1e-3;
        uint8_t no_change_count = 0;
        uint8_t index_loop = 0; //Tracks the number of times the loop is run
        float inVal;

        while((no_change_count < MS_NUM_NODES) && index_loop < 3*MS_NUM_CONNECT){
            no_change_count = MS_NUM_INPUTS;

            // Calculate the contributions of the added connections and Nodes
            for (uint8_t nodeNum = MS_NUM_INPUTS; nodeNum < MS_NUM_NODES; nodeNum++){
                inVal = 0;
                for (uint8_t connectNum = 0; connectNum < (MS_NUM_CONNECT-2*MS_NUM_INPUTS); connectNum++) {
                    if(nnParams.connectTo[connectNum] == nnParams.node_ID[nodeNum]) {
                        float inValueTemp = 0;
                        for(uint8_t i = 0; i < MS_NUM_NODES; i++){
                            if(nnParams.connectFrom[connectNum] == nnParams.node_ID[i]) {
                                inValueTemp = nnParams.node_out[i];
                            }
                        }
                        inVal = inVal + nnParams.connectWeight[connectNum]*inValueTemp;
                    }
                }
                // Add contribution of the original connections to the output node. Hard coded to two outputs
                if (nnParams.node_ID[nodeNum] == 19) {
                    inVal = inVal + outNodeInput1;
                }
                if (nnParams.node_ID[nodeNum] == 20) {
                    inVal = inVal + outNodeInput2;
                }
                // Compare new node output with old output
                inVal = activationFunction(inVal);
                if ((inVal-nnParams.node_out[nodeNum])*(inVal-nnParams.node_out[nodeNum]) < no_change_threshold) {
                    no_change_count = no_change_count + 1;
                }
                nnParams.node_out[nodeNum] = inVal;
            }
            index_loop++;
        }
    }

    // Return the output values:
    float outputs[MS_NUM_OUTPUTS];
    for(uint8_t numOutputs = 0; numOutputs < MS_NUM_OUTPUTS; numOutputs++){
        outputs[numOutputs] = nnParams.node_out[nnParams.outputIndex[numOutputs]];
    }
    
    /* Converting the NN outputs to velocities */
    float theta;
    if (outputs[0] != 0) {
        theta = atanf(fabs(outputs[1]/outputs[0]));
    } else {
    	theta = M_PI/2;
    }
    
    outputs[0] = outputs[0]*MS_MAX_VEL*cosf(theta);
    outputs[1] = outputs[1]*MS_MAX_VEL*sinf(theta);

    // THIS IS FOR GUIDED MODE
    // guidance_h_set_guided_body_vel(outputs[0],outputs[1]);

    guidance_h_set_guided_vel(outputs[0],outputs[1]); 

    // return 0;
}

void ageMS(void){
    /* Conversion between coordinate systems */
    // float a = 0.827559;
    // float b = 0.5613786;
    // float c = -3.903735;
    // float d = 1.0823;

    for(uint8_t x = 0; x < MS_LENGTH/MS_GRID_RES; x ++) {
        for (uint8_t y = 0; y < MS_LENGTH/MS_GRID_RES; y ++) {
            if (msParams.MS[y][x] != 0){
                msParams.MS[y][x] = msParams.MS[y][x] -1;
                if(msParams.MS[y][x] < 1){
                    msParams.MS[y][x] = 1;
                }
            }
        }
    }
    for (uint8_t agentNum = 0; agentNum < MS_SWARM_SIZE; agentNum++){
        uint8_t currentCell_x;
        uint8_t currentCell_y;
        if(agentNum == MS_CURRENT_ID){
            struct EnuCoor_f *pos = stateGetPositionEnu_f();
            // float tempX=(*pos).x;
            // float tempY=(*pos).y;
            // (*pos).x = a*tempX+b*tempY+c;
            // (*pos).y = -b*tempX+a*tempY+d;

            currentCell_x = (uint8_t) ((*pos).x/MS_GRID_RES);
            currentCell_y = (uint8_t) ((*pos).y/MS_GRID_RES);
        }
        else{
            currentCell_x = (uint8_t) (msParams.uavs[agentNum].x/MS_GRID_RES);
            currentCell_y = (uint8_t) (msParams.uavs[agentNum].y/MS_GRID_RES);
        }
        if(msParams.MS[currentCell_y][currentCell_x] != 0) {
            msParams.MS[currentCell_y][currentCell_x] = 100;
        }
    }
}

void neural_network_init(void) {
    /** Initialize the Mission Space age grid */
    for (uint8_t x = 0; x < MS_LENGTH/MS_GRID_RES; x++){
        for (uint8_t y = 0; y < MS_BREDTH/MS_GRID_RES; y++) {
            if( y == 0 || y == MS_BREDTH/MS_GRID_RES-MS_GRID_RES) {
                msParams.MS[y][x] = 0;
            }
            else if(x == 0 || x == MS_LENGTH/MS_GRID_RES - MS_GRID_RES){
                msParams.MS[y][x] = 0;
            }
            else {
                msParams.MS[y][x] = 1;
            }
        }
    }
    
    msParams.sensorRange = MS_SENSOR_RANGE;
    
    /* Starting positions of the Drones */
    msParams.uavs[0].x = 3.5;
    msParams.uavs[0].y = 3.5;
    msParams.uavs[1].x = 0.5;
    msParams.uavs[1].y = 0.5;

    // Neural Network Weights
    uint8_t assign[24] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,25,28,116,20,37,19};
    memcpy(nnParams.node_ID, assign, MS_NUM_NODES*sizeof(uint8_t));

    uint8_t assign2[2] = {21,23};
    memcpy(nnParams.outputIndex, assign2, 2*sizeof(uint8_t));

    float connectionsInit[36] = {5.283783269,-1.299470374,5.41218604,-7.234246276,-3.300530603,-0.932467532,-6.052529676,1.704697964,-6.358833382,8,8,-2.854298496,-8,-1.582748166,-1.914545222,8,4.355598982,2.627215527,5.530937664,-4.293201699,-7.912133623,-4.560515003,-2.432927559,6.428544479,3.236962558,7.434877323,-6.141023524,-2.004435827,-3.654535521,-7.174535472,-7.00926667,6.136937081,0.3445915,2.93038681,2.068986956,5.76611966};
    memcpy(nnParams.connectionsInit, connectionsInit, 36*sizeof(float));

    float connect[8] = {-6.640326374,0.122216172,3.519665279,-2.839629468,-1.236220431,3.307892439,-0.043001854,-8};
    memcpy(nnParams.connectWeight, connect, 8*sizeof(float));

    uint8_t connectTo[8] = {25,20,28,20,37,19,116,37};
    memcpy(nnParams.connectTo, connectTo, 8*sizeof(uint8_t));

    uint8_t connectFrom[8] = {14,25,18,28,13,37,13,116};
    memcpy(nnParams.connectFrom, connectFrom, 8*sizeof(uint8_t));   
}


void neural_network_periodic(void) {
    calcNN();
}