#include "CtrlStruct_gr4.h"
#include "namespace_ctrl.h"

NAMESPACE_INIT(ctrlGr4);

/*! \brief initialize the controller structure
 *
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure initialized
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs, CtrlOut *py_outputs)
{
	CtrlStruct *cvs;

	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));

	cvs->inputs  = inputs;
	cvs->outputs = outputs;
	cvs->py_outputs = py_outputs;

    cvs->tprev=cvs->inputs->t;
    cvs->tactual =cvs->inputs->t;
    cvs->period = 0;
    cvs->integ_err_l_prev=0;
    cvs->integ_err_r_prev=0;
    cvs->err_xyt[0]=0;
    cvs->err_xyt[1]=0;
    cvs->err_xyt[2]=0;

    cvs->wheel_demands[0] = 0;
    cvs->wheel_demands[1] = 0;
    cvs->outputs->wheel_commands[0] = 0;
    cvs->outputs->wheel_commands[1] = 0;
    cvs->outputs->tower_command = 0;

    cvs->stateGlobal = 0; //set state to calibration to start with for now
    cvs->stateCalibration = 0; // to distinguish the different steps for the calibration
    cvs->stateNavigation = 0;
    cvs->stateAction = 0; // to distinguish the different actions to be performed

    //initialization of the beacon array [0] = distance, [1] = direction, [2] = radius of influence
    cvs->pos_beacon_disdirray = new double* [3];
    for(int i=0; i<3 ; i++) {
        cvs->pos_beacon_disdirray[i] = new double[inputs->nb_opponents];
    }

	return cvs;
}

/*! \brief release controller main structure memory
 *
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
	free(cvs);
}

NAMESPACE_CLOSE();
