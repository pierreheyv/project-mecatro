#ifndef OBSTACLE_AVOIDANCE_H_INCLUDED
#define OBSTACLE_AVOIDANCE_H_INCLUDED

#include "struct.h"
#include <math.h>

#define WIDTH 0.5 //width between two wheels
#define DECFP 0.5 //distance between de center of mobility and the point of application of the potential field force (scheme)
#define KRA 0.5 //coef of rotation for robot avoid (see in code)
#define KFA 0.5 //coef of force for robot avoid (see in code)

void avoid(CtrlStruct *structure, Map *mymap);

#endif // OBSTACLE_AVOIDANCE_H_INCLUDED
