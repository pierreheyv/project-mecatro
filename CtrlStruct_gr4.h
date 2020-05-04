/*!
 * \file CtrlStruct_gr2.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR4_H_
#define _CTRL_STRUCT_GR4_H_

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include "mapandpath_gr4.h"
#include <stdlib.h>
#include <math.h>
#include <cmath>

#define NBDATAPRED 4 //nb of data stored for predictive potential field

NAMESPACE_INIT(ctrlGr4);

/// Main controller structure
typedef struct CtrlStruct
{
	// From basic code
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs
	CtrlOut *py_outputs; ///< python controller outputs

	//base variables
	int startingSide;

	// Speed controller variables
	double integ_err_l_prev;
	double integ_err_r_prev;
	double prev_err_l;
	double prev_err_r;
	double wheel_demands[2];

	//odometry variable
	double tprev;
	double tactual;
    double period;
	double position_xyt[3];
	double vit_prev[2];
	double err_xyt[3];
	double err_aera;

	//action variables
	int nbitems;
	int nbtargets;
	double startingActiontime; //time when starting action

	//state variables
	int stateGlobal;
	int stateCalibration;
	int stateNavigation;
	int stateAction;//state used for several actions

	//navigation variables
	Map* mymap; //map object described in mapandpath.h
    int navigmode;
	double **pos_beacon_disdirray;//the beacon array [0] = distance, [1] = direction, [2] = radius of influence
	double ***prev_pos_beacon_disdirray;//table of previous beacon information

	//simuvariables
	int piston_state;//poucentage of released piston

	//supplementary input (from sensors
    int piston_out;
    int piston_in;

} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs, CtrlOut *py_outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
