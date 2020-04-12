#ifndef _SPEED_CONTROLLER_GR4_H_
#define _SPEED_CONTROLLER_GR4_H_

#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
#include "sensorInformation_gr4.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cmath>

#define KR 50
#define KL 50
#define WIDTH 0.09
#define WRADIUS 0.03

NAMESPACE_INIT(ctrlGr4);
//////////////////////////////////////////////////////////////////////////////////
//////////////////////      Speed controller         /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void controller_speed_loop(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif // _SPEED_CONTROLLER_GR4_H_
