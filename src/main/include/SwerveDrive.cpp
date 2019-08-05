//Swerve kinda
#pragma once

#include "Robot.h"
#include "JoystickControls.cpp"
#include "Move.cpp"

#include <frc/drive/Vector2d.h>

#include <iostream>

static const float SPEED = 0.5;
static const int ENCODERS = 48;
static const int BUFFER = 25;

/*
Modes:
0 = LStick to turn wheels, Triggers to accelerate/deccelerate
1 = RStick to move
2 = Turn constantly
3 = Controllable turning
4 = 1+3+vectors
5 = WPILib vectors (supposedly the best one)
*/

/*
Vector math explanation:
There are two vectors for each wheel for each movement: one to move the robot, and one to spin it
The first vector (vector1 or driveVector) is the same for each wheel, as the robot moves the same with every wheel
The second vector (vector2 or turnVector) is the same for each wheel, but rotated 90 degrees for each wheel, as the robot needs to spin
To spin, the wheels need to move at 90 degree angles to each other
These are then combined and scaled down by /2 to normalize the wheels so they have speeds that the robot can read but also are still the right direction
This means that the wheels find where they have to go for one "unit motion" at that moment and then they all go in that direction to move properly
This is then scaled into an encoder value and a magnitude for the motors to move to

TL;DR: One vector per wheel to move the robot and one to spin the robot. Add them together, scale it into the motors, and boom swerve.
*/
void Robot::swerveDrive(int mode) {
    //Keep hall values between -ENCODERS and ENCODERS (Note: if sensors are not properly aligning, this may have to be changed)
    if ((*flHall).Get() <= -ENCODERS || (*flHall).Get() >= ENCODERS)
        (*flHall).Reset();
    if ((*frHall).Get() <= -ENCODERS || (*frHall).Get() >= ENCODERS)
        (*frHall).Reset();
    if ((*blHall).Get() <= -ENCODERS || (*blHall).Get() >= ENCODERS)
        (*blHall).Reset();
    if ((*brHall).Get() <= -ENCODERS || (*brHall).Get() >= ENCODERS)
        (*brHall).Reset();
    
    //Define variables
    int targetEncoder[4];
    float targetSpeed[4], turnMagnitude[4];
    float vector1[2], vector2[4][2], finalVector[4][2];
    frc::Vector2d *driveVector = new frc::Vector2d;
    frc::Vector2d *turnVector = new frc::Vector2d;
    frc::Vector2d *sumVector = new frc::Vector2d;

    //Calculate joystick angles
    float lAngle = angleCalc((*control1).GetX(LHand), 0-(*control1).GetY(LHand));
    float rAngle = angleCalc((*control1).GetX(RHand), 0-(*control1).GetY(RHand));

    //Mode 0 drive
    if (mode == 0) {
        //Scale the joystick angle into encoder angle
        targetEncoder[0] = (int)(lAngle / 360 * ENCODERS);
        targetEncoder[1] = targetEncoder[0];
        targetEncoder[2] = targetEncoder[0];
        targetEncoder[3] = targetEncoder[0];
        //Find target speed
        targetSpeed[0] = SPEED * (((*control1).GetTriggerAxis(RHand) - (*control1).GetTriggerAxis(LHand)));
        if (targetSpeed[0] < 0.3 && targetSpeed[0] > -0.3) //Stop range
            targetSpeed[0] = 0;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }
    
    //Mode 1 drive (same as 0 but with joystick to calculate speed)
    if (mode == 1) {
        //Scale joystick angle into encoder angle
        targetEncoder[0] = (int)(rAngle / 360 * ENCODERS);
        targetEncoder[1] = targetEncoder[0];
        targetEncoder[2] = targetEncoder[0];
        targetEncoder[3] = targetEncoder[0];
        //Calculate the target speed
        targetSpeed[0] = (distCalc((*control1).GetX(RHand), 0-(*control1).GetY(RHand))) * SPEED;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }

    //Mode 2 angle calculation (test 90 degree turning)
    if (mode == 2) {
        turnMagnitude[0] = 45;
        turnMagnitude[1] = turnMagnitude[0] + 90;
        turnMagnitude[2] = turnMagnitude[1] + 90;
        turnMagnitude[3] = turnMagnitude[2] + 90;
    }

    //Mode 3/4 angle calculation
    if (mode == 3 || mode == 4) {
        //Scale from (-1)-1 onto 0-90, then rotate 90 for each motor
        turnMagnitude[0] = (((*control1).GetX(LHand)) * 45) + 45;
        turnMagnitude[1] = turnMagnitude[0] + 90;
        turnMagnitude[2] = turnMagnitude[1] + 90;
        turnMagnitude[3] = turnMagnitude[2] + 90;
    }
    
    //Mode 2/3 scaling and speed
    if (mode == 2 || mode == 3) {
        //Scale from degrees onto encoder angle
        targetEncoder[0] = (int)(turnMagnitude[0] / 360 * ENCODERS);
        targetEncoder[1] = (int)((turnMagnitude[1]) / 360 * ENCODERS);
        targetEncoder[2] = (int)((turnMagnitude[2]) / 360 * ENCODERS);
        targetEncoder[3] = (int)((turnMagnitude[3]) / 360 * ENCODERS) % ENCODERS; // "% ENCODERS" to loop ENCODERS back to 0
        //Set each motor to move at SPEED constantly
        targetSpeed[0] = SPEED;
        targetSpeed[1] = targetSpeed[0];
        targetSpeed[2] = targetSpeed[0];
        targetSpeed[3] = targetSpeed[0];
    }

    //Mode 4 vector math (inefficient coding but soon to be replaced hopefully by mode 5)
    if (mode == 4) {
        //Yes this could be better done with loops but it's probably gonna get deleted anyways
        //Find yaw vector to move robot without spinning for each wheel
        vector1[0] = (*control1).GetX(RHand);
        vector1[1] = 0-(*control1).GetY(RHand);
        //Scale from degrees to x and y values ranging 0-1 (reverse angleCalc basically)
        vector2[0][0] = cosf(turnMagnitude[0]);
        vector2[0][1] = sinf(turnMagnitude[0]);
        vector2[1][0] = cosf(turnMagnitude[1]);
        vector2[1][1] = sinf(turnMagnitude[1]);
        vector2[2][0] = cosf(turnMagnitude[2]);
        vector2[2][1] = sinf(turnMagnitude[2]);
        vector2[3][0] = cosf(turnMagnitude[3]);
        vector2[3][1] = sinf(turnMagnitude[3]);
        //Add the two vectors for each wheel
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

    //Mode 5 vector math (WPILib vectors)
    if (mode == 5) {
        driveVector->x = control1->GetX(RHand);
        driveVector->y = 0 - control1->GetY(RHand);
        turnVector->x = cosf(((*control1).GetX(LHand) * 45) + 45);
        turnVector->y = sinf(((*control1).GetX(LHand) * 45) + 45);
        for (int i = 0; i < 4; i++) { //For each wheel:
            sumVector->x = (driveVector->x + turnVector->x) / 2; //Add the two vectors to get one final vector
            sumVector->y = (driveVector->y + turnVector->y) / 2;
            targetEncoder[i] = angleCalc(driveVector->x, driveVector->y); //Calculate the angle of this vector
            targetSpeed[i] = driveVector->Magnitude() * SPEED; //Scale the speed of 
            turnVector->Rotate(-90.0); //Rotate the vector clockwise 90 degrees
        }
    }

    //Robot::moveMotors(targetEncoder, targetSpeed, ENCODERS, SPEED, BUFFER);
    
    //Align wheels
    //if (((*flHall).Get() + ENCODERS) % ENCODERS == targetEncoders[0]) //If we are on target (-10 scales to ENCODERS-10 too), stop the motor
    if ((((*flHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[0] - BUFFER || ((*flHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[0] - BUFFER + ENCODERS) && (((*flHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[0] + BUFFER || ((*flHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[0] + BUFFER - ENCODERS))
        (*flTurn).Set(0);
    //else if ((int)((*flHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[0]) //If it is faster to turn the wheel right to get to target, go right, else go left
    else if ((targetEncoder[0] > ENCODERS / 2 && (*flHall).Get() < targetEncoder[0]) || (targetEncoder[0] < ENCODERS / 2 && (*flHall).Get() > targetEncoder[0])) //If it is faster to turn the wheel right to get to target, go right, else go left
        (*flTurn).Set(SPEED);
    else
        (*flTurn).Set(-SPEED);
    //if (((*frHall).Get() + ENCODERS) % ENCODERS == targetEncoder[1])
    if ((((*frHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[1] - BUFFER || ((*frHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[1] - BUFFER + ENCODERS) && (((*frHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[1] + BUFFER || ((*frHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[1] + BUFFER - ENCODERS))
        (*frTurn).Set(0);
    //else if ((int)((*frHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[1])
    else if ((targetEncoder[1] > ENCODERS / 2 && (*frHall).Get() < targetEncoder[1]) || (targetEncoder[1] < ENCODERS / 2 && (*frHall).Get() > targetEncoder[1]))
        (*frTurn).Set(SPEED);
    else
        (*frTurn).Set(-SPEED);
    //if (((*brHall).Get() + ENCODERS) % ENCODERS == targetEncoder[2])
    if ((((*brHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[2] - BUFFER || ((*brHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[2] - BUFFER + ENCODERS) && (((*brHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[2] + BUFFER || ((*brHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[2] + BUFFER - ENCODERS))
        (*blTurn).Set(0);
    //else if ((int)((*brHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[2])
    else if ((targetEncoder[2] > ENCODERS / 2 && (*brHall).Get() < targetEncoder[2]) || (targetEncoder[2] < ENCODERS / 2 && (*brHall).Get() > targetEncoder[2]))
        (*blTurn).Set(SPEED);
    else
        (*blTurn).Set(-SPEED);
    //if (((*blHall).Get() + ENCODERS) % ENCODERS == targetEncoder[3])
    if ((((*blHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[3] - BUFFER || ((*blHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[3] - BUFFER + ENCODERS) && (((*blHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[3] + BUFFER || ((*blHall).Get() + ENCODERS * 2) % ENCODERS >= targetEncoder[3] + BUFFER - ENCODERS))
        (*brTurn).Set(0);
    //else if ((int)((*blHall).Get() + (ENCODERS * 1.5)) % ENCODERS < targetEncoder[3])
    else if ((targetEncoder[3] > ENCODERS / 2 && (*blHall).Get() < targetEncoder[3]) || (targetEncoder[3] < ENCODERS / 2 && (*blHall).Get() > targetEncoder[3]))
        (*brTurn).Set(SPEED);
    else
        (*brTurn).Set(-SPEED);
    
    //Set drive motors to their respective speeds
    (*flMain).Set(targetSpeed[0]);
    (*frMain).Set(targetSpeed[1]);
    (*brMain).Set(targetSpeed[2]);
    (*blMain).Set(targetSpeed[3]);
    

    //Debugging things:
    std::cout << "Target encoders: " << targetEncoder[0] << "/" << targetEncoder[1] << "/" << targetEncoder[2] << "/" << targetEncoder[3];
    std::cout << "Current encoders: " << (*flHall).Get() << "/" << (*frHall).Get() << "/" << (*brHall).Get() << "/" << (*blHall).Get();
}