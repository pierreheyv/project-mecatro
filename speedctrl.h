#ifndef SPEEDCTRL_H_INCLUDED
#define SPEEDCTRL_H_INCLUDED

#include "IO/COM/CAN/CAN.hh"
#include "IO/COM/SPI/Specific/SPI_CAN.hh"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "struct.h"

#define REDUCTION 19
#define KPHI 0.0261
#define GAINW 0.011
#define KP 0.0299
#define KI 0.2071
#define SAT 22.8//to be adapted !!!
#define CURRENTSAT 0.74//to be adapted !!!
#define RA 7.1//to be adapted !!!

double saturation(double boundaries, double input);
double CtrlOutCalc(double u_ref);
void run_speed_controller(CtrlStruct *theCtrlStruct, CAN *can);

#endif // SPEEDCTRL_H_INCLUDED
