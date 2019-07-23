//Swerve kinda
#pragma once

#include "Robot.h"
#include "JoystickControls.cpp"

#define speed 0.5
#define encoders 48


/*
Modes:
0 = LStick to turn wheels, Triggers to accelerate/deccelerate
1 = RStick to move
2 = Turn (very basic)
3 = Vector-based (i think) turning
4 = 1+3
*/
void Robot::swerveDrive(int mode) {
    //Calculate stuff
    float lAngle = angleCalc((*control1).GetX(LHand), 0-(*control1).GetY(LHand));
    float rAngle = angleCalc((*control1).GetX(RHand), 0-(*control1).GetY(RHand));
    int targetEncoder[4];
    float targetSpeed[4], turnMagnitude[4];
    float vector1[2], vector2[4][2], finalVector[4][2];
    if (mode == 0) {
        targetEncoder[0] = (int)(lAngle / 360 * encoders);
        targetEncoder[1] = targetEncoder[0];
        targetEncoder[2] = targetEncoder[0];
        targetEncoder[3] = targetEncoder[0];
        targetSpeed[0] = speed * (((*control1).GetTriggerAxis(RHand) - (*control1).GetTriggerAxis(LHand)));
        if (targetSpeed[0] < 0.3 && targetSpeed[0] > -0.3)
            targetSpeed[0] = 0;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }
    if (mode == 1 || mode == 4) {
        targetEncoder[0] = (int)(rAngle / 360 * encoders);
        targetEncoder[1] = targetEncoder[0];
        targetEncoder[2] = targetEncoder[0];
        targetEncoder[3] = targetEncoder[0];
        targetSpeed[0] = (distCalc((*control1).GetX(RHand), 0-(*control1).GetY(RHand))) * speed;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }


    if (mode == 2) {
        turnMagnitude[0] = 45;
        turnMagnitude[1] = turnMagnitude[0] + 90;
        turnMagnitude[2] = turnMagnitude[1] + 90;
        turnMagnitude[3] = turnMagnitude[2] + 90;
    }
    if (mode == 3 || mode == 4) {
        turnMagnitude[0] = (((*control1).GetX(LHand) / 32767) * 45) + 45;
        turnMagnitude[1] = turnMagnitude[0] + 90;
        turnMagnitude[2] = turnMagnitude[1] + 90;
        turnMagnitude[3] = turnMagnitude[2] + 90;
    }
    if (mode == 2 || mode == 3) {
        targetEncoder[0] = (int)(turnMagnitude[0] / 360 * encoders);
        targetEncoder[1] = (int)((turnMagnitude[1]) / 360 * encoders);
        targetEncoder[2] = (int)((turnMagnitude[2]) / 360 * encoders);
        targetEncoder[3] = (int)((turnMagnitude[3]) / 360 * encoders) % 40; // "% 40" to loop 40 back to 0
        targetSpeed[0] = speed;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }

    if (mode == 4) {
        //vector math shit
        vector1[0] = (*control1).GetX(RHand);
        vector1[1] = 0-(*control1).GetY(RHand);
        vector1[0] /= 32767;
        vector1[1] /= 32767;
        vector2[0][0] = cosf(turnMagnitude[0]);
        vector2[0][1] = sinf(turnMagnitude[0]);
        vector2[1][0] = cosf(turnMagnitude[1]);
        vector2[1][1] = sinf(turnMagnitude[1]);
        vector2[2][0] = cosf(turnMagnitude[2]);
        vector2[2][1] = sinf(turnMagnitude[2]);
        vector2[3][0] = cosf(turnMagnitude[3]);
        vector2[3][1] = sinf(turnMagnitude[3]);
        finalVector[0][0] = vector1[0] + vector2[0][0];
        finalVector[0][1] = vector1[1] + vector2[0][1];
        finalVector[1][0] = vector1[0] + vector2[1][0];
        finalVector[1][1] = vector1[1] + vector2[1][1];
        finalVector[2][0] = vector1[0] + vector2[2][0];
        finalVector[2][1] = vector1[1] + vector2[2][1];
        finalVector[3][0] = vector1[0] + vector2[3][0];
        finalVector[3][1] = vector1[1] + vector2[3][1];

        //vectors --> angles & speeds
        targetEncoder[0] = angleCalc(finalVector[0][0], finalVector[0][1]);
        targetEncoder[1] = angleCalc(finalVector[1][0], finalVector[1][1]);
        targetEncoder[2] = angleCalc(finalVector[2][0], finalVector[2][1]);
        targetEncoder[3] = angleCalc(finalVector[3][0], finalVector[3][1]);
        targetSpeed[0] = distCalc(finalVector[0][0], finalVector[0][1]) / 2 * speed;
        targetSpeed[1] = distCalc(finalVector[1][0], finalVector[1][1]) / 2 * speed;
        targetSpeed[2] = distCalc(finalVector[2][0], finalVector[2][1]) / 2 * speed;
        targetSpeed[3] = distCalc(finalVector[3][0], finalVector[3][1]) / 2 * speed;
    }


    //Align wheels
    if ((*flHall).Get() == targetEncoder[0])
        (*flTurn).Set(0);
    else if (((*flHall).Get() + (encoders / 2)) % encoders > targetEncoder[0])
        (*flTurn).Set(speed);
    else
        (*flTurn).Set(-speed);
    if ((*frHall).Get() == targetEncoder[1])
        (*frTurn).Set(0);
    else if (((*frHall).Get() + (encoders / 2)) % encoders > targetEncoder[1])
        (*frTurn).Set(speed);
    else
        (*frTurn).Set(-speed);
    if ((*blHall).Get() == targetEncoder[2])
        (*blTurn).Set(0);
    else if (((*blHall).Get() + (encoders / 2)) % encoders > targetEncoder[2])
        (*blTurn).Set(speed);
    else
        (*blTurn).Set(-speed);
    if ((*brHall).Get() == targetEncoder[3])
        (*brTurn).Set(0);
    else if (((*brHall).Get() + (encoders / 2)) % encoders > targetEncoder[3])
        (*brTurn).Set(speed);
    else
        (*brTurn).Set(-speed);
    
    //Move
    (*flMain).Set(targetSpeed[0]);
    (*frMain).Set(targetSpeed[1]);
    (*brMain).Set(targetSpeed[2]);
    (*blMain).Set(targetSpeed[3]);
}