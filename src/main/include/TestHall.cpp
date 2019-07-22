//Basic program to test Hall Effect sensor
#pragma once

#include "Robot.h"

void Robot::testHall() {
  if ((*control1).GetAButton() == 1) {
    while (((*flHall).Get()) < 10 && ((*flHall).Get()) > -10)
      (*flTurn).Set(0.5);
    (*flTurn).Set(0);
  }
  if ((*control1).GetBButton() == 1) {
    while (((*flHall).Get()) < 10 && ((*flHall).Get()) > -10)
      (*flTurn).Set(-0.5);
    (*flTurn).Set(0);
  }
  if ((*control1).GetXButton() == 1) {
    while (((*frHall).Get()) < 10 && ((*frHall).Get()) > -10)
      (*frTurn).Set(0.5);
    (*frTurn).Set(0);
  }
  if ((*control1).GetYButton() == 1) {
    while (((*frHall).Get()) < 10 && ((*frHall).Get()) > -10)
      (*frTurn).Set(-0.5);
    (*frTurn).Set(0);
  }
}