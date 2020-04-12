#ifndef _NAVIGATION_GR4_H_
#define _NAVIGATION_GR4_H_

#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
#include "mapandpath_gr4.h"
#include <stdlib.h>

#include <stdio.h>
#include <math.h>
#include <cmath>

#define KR 50
#define KL 50

#define DECFP 0.03
#define ROBOTRADIUS 0.14
#define KRA 0.1
#define WIDENESS 0.07
#define KFA 0.1
#define WIDTH 0.09

NAMESPACE_INIT(ctrlGr4);

//////////////////////////////////////////////////////////////////////////////////
///////////////////////       Navigation commands        /////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void navigation(CtrlStruct *cvs);

void middle_controller(CtrlStruct *cvs, double x, double y);
void rot(CtrlStruct *cvs, double angle);

void potential_field(CtrlStruct *structure, Map *mymap);


NAMESPACE_CLOSE();
#endif // _NAVIGATION_GR4_H_

