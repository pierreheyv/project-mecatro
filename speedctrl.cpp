#include "speedctrl.h"

double saturation(double boundaries, double input)
{

    if(input < -boundaries)
    {
        printf("saturation <  0 \n");
        return (-boundaries);
    }

    else if(input < boundaries)
    {
        return input;
    }
    else
    {
        printf("Saturation \n");
        return boundaries;
    }
}

double CtrlOutCalc(double u_ref)
{
    return 100*u_ref/SAT;
}


void run_speed_controller(CtrlStruct* theCtrlStruct)
{

    double omega_ref[2];
    omega_ref[0] = theCtrlStruct->theUserStruct->wantedspeedl;
    omega_ref[1] = theCtrlStruct->theUserStruct->wantedspeedr;

    double mesure_l = theCtrlStruct->theCtrlIn->l_wheel_speed;
    double mesure_r = theCtrlStruct->theCtrlIn->r_wheel_speed;

    double period = theCtrlStruct->theUserStruct->period;

    theCtrlStruct->theUserStruct->Error_omega_l = ((omega_ref[L_ID]-mesure_l)*REDUCTION*5.6);
    theCtrlStruct->theUserStruct->Error_omega_r = ((omega_ref[R_ID]-mesure_r)*REDUCTION*5.6);//code de base avec * REDUCTION

    theCtrlStruct->theUserStruct->Error_int_omega_l += theCtrlStruct->theUserStruct->Error_omega_l * period;
    theCtrlStruct->theUserStruct->Error_int_omega_r += theCtrlStruct->theUserStruct->Error_omega_r * period;

    double U_ref_l = KP*(theCtrlStruct->theUserStruct->Error_omega_l) + KI*theCtrlStruct->theUserStruct->Error_int_omega_l;
    double U_ref_r = KP*(theCtrlStruct->theUserStruct->Error_omega_r) + KI*theCtrlStruct->theUserStruct->Error_int_omega_r;

    double Ul = U_ref_l+KPHI*mesure_l;
    double Ur = U_ref_r+KPHI*mesure_r;


    double applied_u_l = saturation(SAT, Ul);
    double applied_u_r = saturation(SAT, Ur);


    theCtrlStruct->theCtrlOut->wheel_commands[0] = CtrlOutCalc(applied_u_l);
    theCtrlStruct->theCtrlOut->wheel_commands[1] = CtrlOutCalc(applied_u_r);
}

