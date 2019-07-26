//Swerve kinda
#pragma once

#include "Robot.h"
#include "JoystickControls.cpp"

#include <frc/drive/Vector2d.h>

#include <iostream>

static const float SPEED = 0.5;
static const int ENCODERS = 48;


/*
Modes:
0 = LStick to turn wheels, Triggers to accelerate/deccelerate
1 = RStick to move
2 = Turn (very basic)
3 = Vector-based (i think) turning
4 = 1+3
5 = WPILib vectors
*/
void Robot::swerveDrive(int mode) {
    //Keep hall values between -ENCODERS and ENCODERS
    if ((*flHall).Get() <= -ENCODERS || (*flHall).Get() >= ENCODERS)
        (*flHall).Reset();
    if ((*frHall).Get() <= -ENCODERS || (*frHall).Get() >= ENCODERS)
        (*frHall).Reset();
    if ((*blHall).Get() <= -ENCODERS || (*blHall).Get() >= ENCODERS)
        (*blHall).Reset();
    if ((*brHall).Get() <= -ENCODERS || (*brHall).Get() >= ENCODERS)
        (*brHall).Reset();
    
    //Calculate stuff
    float lAngle = angleCalc((*control1).GetX(LHand), 0-(*control1).GetY(LHand));
    float rAngle = angleCalc((*control1).GetX(RHand), 0-(*control1).GetY(RHand));
    int targetEncoder[4];
    float targetSpeed[4], turnMagnitude[4];
    float vector1[2], vector2[4][2], finalVector[4][2];
    frc::Vector2d *driveVector = new frc::Vector2d;
    frc::Vector2d *turnVector = new frc::Vector2d;
    frc::Vector2d *sumVector = new frc::Vector2d;
    if (mode == 0) {
        targetEncoder[0] = (int)(lAngle / 360 * ENCODERS);
        targetEncoder[1] = targetEncoder[0];
        targetEncoder[2] = targetEncoder[0];
        targetEncoder[3] = targetEncoder[0];
        targetSpeed[0] = SPEED * (((*control1).GetTriggerAxis(RHand) - (*control1).GetTriggerAxis(LHand)));
        if (targetSpeed[0] < 0.3 && targetSpeed[0] > -0.3)
            targetSpeed[0] = 0;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }
    if (mode == 1 || mode == 4) {
        targetEncoder[0] = (int)(rAngle / 360 * ENCODERS);
        targetEncoder[1] = targetEncoder[0];
        targetEncoder[2] = targetEncoder[0];
        targetEncoder[3] = targetEncoder[0];
        targetSpeed[0] = (distCalc((*control1).GetX(RHand), 0-(*control1).GetY(RHand))) * SPEED;
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
        turnMagnitude[0] = (((*control1).GetX(LHand)) * 45) + 45;
        turnMagnitude[1] = turnMagnitude[0] + 90;
        turnMagnitude[2] = turnMagnitude[1] + 90;
        turnMagnitude[3] = turnMagnitude[2] + 90;
    }
    if (mode == 2 || mode == 3) {
        targetEncoder[0] = (int)(turnMagnitude[0] / 360 * ENCODERS);
        targetEncoder[1] = (int)((turnMagnitude[1]) / 360 * ENCODERS);
        targetEncoder[2] = (int)((turnMagnitude[2]) / 360 * ENCODERS);
        targetEncoder[3] = (int)((turnMagnitude[3]) / 360 * ENCODERS) % ENCODERS; // "% ENCODERS" to loop ENCODERS back to 0
        targetSpeed[0] = SPEED;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }

    if (mode == 4) {
        //Yes this could be better done with loops but it's probably gonna get deleted anyways
        //vector math crap
        vector1[0] = (*control1).GetX(RHand);
        vector1[1] = 0-(*control1).GetY(RHand);
        vector2[0][0] = cosf(turnMagnitude[0]); //Scale from degrees to values 0-1 for X and Y
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
        targetSpeed[0] = distCalc(finalVector[0][0], finalVector[0][1]) / 2 * SPEED;
        targetSpeed[1] = distCalc(finalVector[1][0], finalVector[1][1]) / 2 * SPEED;
        targetSpeed[2] = distCalc(finalVector[2][0], finalVector[2][1]) / 2 * SPEED;
        targetSpeed[3] = distCalc(finalVector[3][0], finalVector[3][1]) / 2 * SPEED;
    }

    //Here goes
    if (mode == 5) {
        driveVector->x = control1->GetX(RHand);
        driveVector->y = 0 - control1->GetY(RHand);
        turnVector->x = cosf(((*control1).GetX(LHand) * 45) + 45);
        turnVector->y = sinf(((*control1).GetX(LHand) * 45) + 45);
        for (int i = 0; i < 4; i++) {
            sumVector->x = (driveVector->x + turnVector->x) / 2;
            sumVector->y = (driveVector->y + turnVector->y) / 2;
            targetEncoder[i] = angleCalc(driveVector->x, driveVector->y);
            targetSpeed[i] = driveVector->Magnitude() * SPEED;
            driveVector->Rotate(90.0);
        }
    }


    //Align wheels
    if (((*flHall).Get() + ENCODERS) % ENCODERS == targetEncoder[0])
        (*flTurn).Set(0);
    else if ((int)((*flHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[0])
        (*flTurn).Set(SPEED);
    else
        (*flTurn).Set(-SPEED);
    if (((*frHall).Get() + ENCODERS) % ENCODERS == targetEncoder[1])
        (*frTurn).Set(0);
    else if ((int)((*frHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[1])
        (*frTurn).Set(SPEED);
    else
        (*frTurn).Set(-SPEED);
    if (((*blHall).Get() + ENCODERS) % ENCODERS == targetEncoder[2])
        (*blTurn).Set(0);
    else if ((int)((*blHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[2])
        (*blTurn).Set(SPEED);
    else
        (*blTurn).Set(-SPEED);
    if (((*brHall).Get() + ENCODERS) % ENCODERS == targetEncoder[3])
        (*brTurn).Set(0);
    else if ((int)((*brHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[3])
        (*brTurn).Set(SPEED);
    else
        (*brTurn).Set(-SPEED);
    
    //Move
    (*flMain).Set(targetSpeed[0]);
    (*frMain).Set(targetSpeed[1]);
    (*brMain).Set(targetSpeed[2]);
    (*blMain).Set(targetSpeed[3]);


    //Debugging things:
    std::cout << "Target encoders: " << targetEncoder[0] << "/" << targetEncoder[1] << "/" << targetEncoder[2] << "/" << targetEncoder[3];
    std::cout << "Current encoders: " << (*flHall).Get() << "/" << (*frHall).Get() << "/" << (*brHall).Get() << "/" << (*blHall).Get();
}