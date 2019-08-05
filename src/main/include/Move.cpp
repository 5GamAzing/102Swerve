//Align the wheels and move the motors

#pragma once

#include "Robot.h"

/*
Arguments:
int *targetEncoders: an array with 4 values corresponding to the target encoder values for fL, fR, bR, and bL (in that order) (range from -ENCODERS to ENCODERS)
float *targetSpeeds: an array with 4 values corresponding to the speeds at which to move each drive motor at (same order as above) (range from -1 to 1)
int ENCODERS: the amount of encoders in a full rotation
int SPEED: the speed at which motors should turn the wheel at (range from 0.1 to 1)
*/

void Robot::moveMotors(int *targetEncoders, float *targetSpeeds, int ENCODERS, int SPEED, int BUFFER) {
    //Align wheels
    if (((*flHall).Get() + ENCODERS) % ENCODERS == targetEncoders[0]) //If we are on target (-10 scales to ENCODERS-10 too), stop the motor
        (*flTurn).Set(0);
    else if ((int)((*flHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoders[0]) //If it is faster to turn the wheel right to get to target, go right, else go left
        (*flTurn).Set(SPEED);
    else
        (*flTurn).Set(-SPEED);
    if (((*frHall).Get() + ENCODERS) % ENCODERS == targetEncoders[1])
        (*frTurn).Set(0);
    else if ((int)((*frHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoders[1])
        (*frTurn).Set(SPEED);
    else
        (*frTurn).Set(-SPEED);
    if (((*blHall).Get() + ENCODERS) % ENCODERS == targetEncoders[2])
        (*blTurn).Set(0);
    else if ((int)((*blHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoders[2])
        (*blTurn).Set(SPEED);
    else
        (*blTurn).Set(-SPEED);
    if (((*brHall).Get() + ENCODERS) % ENCODERS == targetEncoders[3])
        (*brTurn).Set(0);
    else if ((int)((*brHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoders[3])
        (*brTurn).Set(SPEED);
    else
        (*brTurn).Set(-SPEED);
    
    //Set drive motors to their respective speeds
    (*flMain).Set(targetSpeeds[0]);
    (*frMain).Set(targetSpeeds[1]);
    (*brMain).Set(targetSpeeds[2]);
    (*blMain).Set(targetSpeeds[3]);
}