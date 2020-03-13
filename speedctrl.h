#ifndef SPEEDCTRL_H_INCLUDED
#define SPEEDCTRL_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "struct.h"

#define REDUCTION 19
#define KPHI 0.0261
#define GAINW 0.011
#define KP 0.0299
#define KI 0.2071
#define SAT 22.8
#define CURRENTSAT 4//0.74
#define RA 7.1

double saturation(double boundaries, double input);
double CtrlOutCalc(double u_ref);
void run_speed_controller(CtrlStruct *theCtrlStruct);

#endif // SPEEDCTRL_H_INCLUDED
