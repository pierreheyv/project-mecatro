/*!
 * \file speed_controller_gr4.cc
 * \brief
 */

#include "speed_controller_gr4.h"
#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"


NAMESPACE_INIT(ctrlGr4);

//////////////////////////////////////////////////////////////////////////////////
//////////////////////      Speed controller         /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void controller_speed_loop(CtrlStruct *cvs)
{
    cvs->period = fabs(cvs->tactual - cvs->tprev);

    double omega_ref_m_r = cvs->wheel_demands[0]*14;
    double omega_ref_m_l = cvs->wheel_demands[1]*14;

    double ki=0.066;
    double kp=0.017;//0.025
    double kPhi=0.0261;
    double K=0.9998;

    double lspeed = cvs->inputs->l_wheel_speed;
    double rspeed = cvs->inputs->r_wheel_speed;

    double omega_m_l = lspeed*14;
    double omega_m_r = rspeed*14;

    double err_l = omega_ref_m_l-omega_m_l;
    double err_r = omega_ref_m_r-omega_m_r;

    double integ_err_l = err_l*(cvs->period) + cvs->integ_err_l_prev;
    double integ_err_r = err_r*(cvs->period) + cvs->integ_err_r_prev;

    if (integ_err_l>0.95*24)
    {
        integ_err_l = 0.95*24;
    }
    if (integ_err_r>0.95*24)
    {
        integ_err_r = 0.95*24;
    }

    double vl = kp*err_l + ki*integ_err_l;
    double vr = kp*err_r + ki*integ_err_r;

    double Vl = vl+kPhi*omega_m_l/K;
    double Vr = vr+kPhi*omega_m_r/K;

    cvs->integ_err_l_prev = integ_err_l;
    cvs->integ_err_r_prev = integ_err_r;

    double Comm_l;
    double Comm_r;

    if (Vl>0.95*24)
    {
        Comm_l = 100;
    }
    else
    {
        if (Vl<-0.95*24)
        {
            Comm_l = -100;
        }
        else
        {
            Comm_l = Vl*100/(0.95*24);
        }
    }

    if (Vr>0.95*24)
    {
        Comm_r = 100;
    }
    else
    {
        if (Vr<-0.95*24)
        {
            Comm_r = -100;
        }
        else
        {
            Comm_r = Vr*100/(0.95*24);
        }
    }

    cvs->outputs->wheel_commands[0] = Comm_r;
    cvs->outputs->wheel_commands[1] = Comm_l;
    cvs->tprev=cvs->tactual;

    return;
}

NAMESPACE_CLOSE();
