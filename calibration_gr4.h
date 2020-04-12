#ifndef _CALIBRATION_GR4_H_
#define _CALIBRATION_GR4_H_

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
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
//////////////////////////       Calibration        //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void calibration(CtrlStruct *cvs);
void initPos(CtrlStruct *cvs, int wallnb);

NAMESPACE_CLOSE();


#endif // CALIBRATION_GR4_H
