#ifndef _ODOMETRY_GR4_H_
#define _ODOMETRY_GR4_H_

#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
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
//////////////////////////       Odometry        /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void odometrie(CtrlStruct *cvs);

//////////////////////////////////////////////////////////////////////////////////
//////////       Infos from detected opponent's robot beacons       //////////////
//////////////////////////////////////////////////////////////////////////////////
void dist_to_beam(CtrlStruct *cvs);


NAMESPACE_CLOSE();
#endif // ODOMETRY_H
