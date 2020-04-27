#ifndef _ODOMETRY_GR4_H_
#define _ODOMETRY_GR4_H_

#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
#include <stdlib.h>

#include <stdio.h>
#include <math.h>
#include <cmath>

#define WIDTH 0.09 //robot width
#define WRADIUS 0.03 //wheel radius
#define DANGERDIST 0.1 //distance from opponent when pot navigation is needed
#define OUTOFDANGERDIST 0.2 //distance from opponennt where pot navigation is no more needed


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
