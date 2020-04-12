/*!
 * \file ctrl_main_gr4.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR4_H_
#define _CTRL_MAIN_GR4_H_

#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
#include <stdlib.h>

#include "mapandpath_gr4.h"
#include "calibration_gr4.h"
#include "navigation_gr4.h"
#include "sensorInformation_gr4.h"
#include "performActions_gr4.h"
#include "speed_controller_gr4.h"


#include <stdio.h>
#include <math.h>
#include <cmath>

#define WIDTH 0.09
#define WRADIUS 0.03
#define TIMEMARGIN 20

NAMESPACE_INIT(ctrlGr4);

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);

void timeisover(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif
