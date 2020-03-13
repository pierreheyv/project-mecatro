#ifndef IA_H_INCLUDED
#define IA_H_INCLUDED

#include "struct.h"

void computecmd(CtrlStruct *theCtrlStruct);
void cmdtype(CtrlStruct *theCtrlStruct); //chooses the cmd type (manual = 0, avoidance = 1, test =  2, ...)
void takenext_instru(CtrlStruct *theCtrlStruct);

#endif // IA_H_INCLUDED
