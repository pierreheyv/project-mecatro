#ifndef IA_H_INCLUDED
#define IA_H_INCLUDED

#include "struct.h"
#include "positionctrl.h"
#include "speedctrl.h"
#include "pathplanning.h"
#include "obstacle_avoidance.h"

//middle level controller cst
#define KR 0.5 //coef de rotation
#define KL 1 //coef linear
#define WIDTH 0.5 //width between two wheels or odometer (to think)

//high level controller
void computecmd(CtrlStruct *theCtrlStruct, Map *mymap);
void takenext_dest(CtrlStruct *theCtrlStruct, double dest[]);//take the next destination in a txt file
void pathplanning(CtrlStruct *theCtrlStruct); //write list of position to get to the objective

//calibration
void calib(CtrlStruct *theCtrlStruct);
void initpos(CtrlStruct *theCtrlStruct, int wallnb);//put position x or y to 0

//middle level controller
void middle_controller(CtrlStruct *structure, double objpos[3]);
void rot(CtrlStruct *structure, double diffangle);

//action
void nodeaction(CtrlStruct *theCtrlStruct, Map *mymap);

#endif // IA_H_INCLUDED
