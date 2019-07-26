//Basic program to test Hall Effect sensor
#pragma once

#include "Robot.h"

void Robot::testHall() {
  if ((*control1).GetAButton() == 1) {
    while (((*flHall).Get()) > -10)
      (*flTurn).Set(1);
    (*flTurn).Set(0);
  }
  if ((*control1).GetBButton() == 1) {
    while (((*flHall).Get()) < 10)
      (*flTurn).Set(-1);
    (*flTurn).Set(0);
  }
  if ((*control1).GetXButton() == 1) {
    (*frTurn).Set(1);
  }
  else if ((*control1).GetYButton() == 1) {
    (*frTurn).Set(-1);
  }
  else {
    (*frTurn).Set(0);
  }
}