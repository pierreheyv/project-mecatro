#ifndef IA_H_INCLUDED
#define IA_H_INCLUDED

#include "struct.h"
#include "positionctrl.h"
#include "speedctrl.h"

void computecmd(CtrlStruct *theCtrlStruct);
void takenext_dest(CtrlStruct *theCtrlStruct, double dest[]);//take the next destination in a txt file
void pathplanning(CtrlStruct *theCtrlStruct); //write list of position to get to the objective

void initpos(CtrlStruct *theCtrlStruct, int wallnb);//put position x or y to 0

#endif // IA_H_INCLUDED
