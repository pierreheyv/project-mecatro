/*!
 * \file calibration_gr4.cc
 * \brief
 */

#include "namespace_ctrl.h"

#include "calibration_gr4.h"
#include "CtrlStruct_gr4.h"

NAMESPACE_INIT(ctrlGr4);

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////       Calibration        //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*! \brief calibration step
 *
 * \param[in] cvs controller main structure
 */
void calibration(CtrlStruct *cvs)
{
    switch(cvs->stateCalibration)
    {
    case 0: // going back
    {
        printf("Going back \n");
        cvs->wheel_demands[0] = -30;
        cvs->wheel_demands[1] = -30;
        if (cvs->inputs->u_switch[0])
        {
            cvs->wheel_demands[0] = 0;
            if (cvs->inputs->u_switch[1])
            {
                cvs->wheel_demands[1] =  0;
                initPos(cvs, 1*(cvs->startingSide));
                cvs->stateCalibration = 1;
            }
        }
        break;
    }

    case 1 : //going forward
    {
        printf("Going forward \n");
        cvs->wheel_demands[0] = 30;
        cvs->wheel_demands[1] = 30;
        if(cvs->startingSide)
        {
            if (cvs->position_xyt[1] >= (-1.37))
            {
                cvs->wheel_demands[0] =  0;
                cvs->wheel_demands[1] =  0;
                cvs->stateCalibration = 2;
            }
        }
        else
        {
            if (cvs->position_xyt[1] <= (1.37))
            {
                cvs->wheel_demands[0] =  0;
                cvs->wheel_demands[1] =  0;
                cvs->stateCalibration = 2;
            }
        }
        break;
    }

    case 2 : // turn to wall corner
    {
        printf("Turn to wall corner \n");
        cvs->wheel_demands[0] = 20*(cvs->startingSide);
        cvs->wheel_demands[1] = -20*(cvs->startingSide);

        double angle = fabs(cvs->position_xyt[2]);
        if (cvs->position_xyt[2] > (M_PI*0.99) && angle < (1.01*M_PI))
        {
            cvs->wheel_demands[0] =  0;
            cvs->wheel_demands[1] =  0;
            cvs->stateCalibration = 3;
        }
        break;
    }
    case 3: //going to the corner wall
    {
        printf("Going to the corner wall \n");
        cvs->wheel_demands[0] = -20;
        cvs->wheel_demands[1] = -20;
        if (cvs->inputs->u_switch[0])
        {
            cvs->wheel_demands[0] = 0;
            if (cvs->inputs->u_switch[1])
            {
                cvs->wheel_demands[1] =  0;
                if (cvs->inputs->u_switch[0])
                {
                    initPos(cvs, 2*(cvs->startingSide));
                    cvs->stateCalibration = 4;
                }
            }
        }
        break;
    }
    case 4 : //going back to start
    {
        printf("Going back to start \n");
        cvs->wheel_demands[0] = 20; //a definir
        cvs->wheel_demands[1] = 20; //a definir
        if (cvs->position_xyt[0] >= -0.3)
        {
            cvs->wheel_demands[0] = 0;
            cvs->wheel_demands[1] = 0;

            cvs->stateCalibration = 0;
            cvs->stateGlobal = 1;  // Fixed to "navigation" for now
        }
        break;
    }
    }
}

/*! \brief position initialisation
 *
 * \param[in] cvs controller main structure, wallnb : number of the wall used for calibration (neg if left side)
 */
void initPos(CtrlStruct *cvs, int wallnb)
{
    if (wallnb == 1)
    {
        cvs->position_xyt[1] = -(1.5 - 0.06);
        cvs->position_xyt[2] = M_PI_2; //M_PI_2;//orientation = 0
    }

    else if (wallnb == 2)
    {
        cvs->position_xyt[0] = -(0.880 - 0.06); //-1.0;
        cvs->position_xyt[2] = 0;
    }

    else if (wallnb == -1)
    {
        cvs->position_xyt[1] = (1.5 - 0.06);
        cvs->position_xyt[2] = - M_PI_2; //M_PI_2;
    }

    else if (wallnb == -2)
    {
        cvs->position_xyt[0] = -(0.880 - 0.06); //-1.0;
        cvs->position_xyt[2] = 0; //M_PI;
    }
}

NAMESPACE_CLOSE();
