#ifndef _PERFORMACTIONS_GR4_H_
#define _PERFORMACTIONS_GR4_H_

#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"
#include "mapandpath_gr4.h"
#include <stdlib.h>

#include <stdio.h>
#include <math.h>
#include <cmath>

#define DEPONODE 1//number assioscieted with the depo in the map generation

NAMESPACE_INIT(ctrlGr4);

//////////////////////////////////////////////////////////////////////////////////
///////////////////////       perform actions        /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void performActions(CtrlStruct *cvs);
void nextobj(Path* mypath);

//////////////////////////////////////////////////////////////////////////////////
///////////////////////       Stop all movements        //////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void stopActions(CtrlStruct *cvs);

NAMESPACE_CLOSE();
#endif // _PERFORMACTIONS_GR4_H_
