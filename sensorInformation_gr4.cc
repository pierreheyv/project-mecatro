/*!
 * \file odometry_gr4.cc
 * \brief
 */

#include "namespace_ctrl.h"

#include "sensorInformation_gr4.h"
#include "CtrlStruct_gr4.h"

#define WIDENESS 0.07

NAMESPACE_INIT(ctrlGr4);

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////       Odometry        /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void odometrie(CtrlStruct *cvs)
{
    double lspeed = cvs->inputs->l_wheel_speed;
    double rspeed = cvs->inputs->r_wheel_speed;

    double lspeed_mean = (lspeed+cvs->vit_prev[0])/2;
    double rspeed_mean = (rspeed+cvs->vit_prev[1])/2;

    double dsr = lspeed_mean*(cvs->period)*WRADIUS;//dsiplacement on left and right wheel
    double dsl = rspeed_mean*(cvs->period)*WRADIUS;

    double ds = (dsr+dsl)/2; //center displacement
    double dt = (dsr-dsl)/(2*WIDTH); //delta angle

    double meant = cvs->position_xyt[2] + dt/2;//mean theta
    double dx = ds*cos(meant);
    double dy = ds*sin(meant);

    cvs->position_xyt[0] = cvs->position_xyt[0] + dx;
    cvs->position_xyt[1] = cvs->position_xyt[1] + dy;
    cvs->position_xyt[2] = (std::fmod((cvs->position_xyt[2] + dt),(2*M_PI)));
    if(cvs->position_xyt[2] < 0){ cvs->position_xyt[2] = cvs->position_xyt[2] + 2*M_PI;}

    cvs->err_xyt[0]=cvs->err_xyt[0]+0.007*dx;
    cvs->err_xyt[1]=cvs->err_xyt[1]+0.007*dy;
    cvs->err_xyt[2]=cvs->err_xyt[2]+0.007*dt;
    cvs->err_aera=M_PI*cvs->err_xyt[0]*cvs->err_xyt[1];


    cvs->vit_prev[0] = lspeed;
    cvs->vit_prev[1] = rspeed;

    printf("posx : %f \n", cvs->position_xyt[0]);
    printf("posy : %f \n", cvs->position_xyt[1]);
    printf("theta : %f\n", cvs->position_xyt[2]);
}

//////////////////////////////////////////////////////////////////////////////////
//////////       Infos from detected opponent's robot beacons       //////////////
//////////////////////////////////////////////////////////////////////////////////

void dist_to_beam(CtrlStruct *cvs){

    printf("\n");

    // WIDENESS = DETECTED wideness of the beacon in [mm]
    if (cvs->inputs->nb_opponents != cvs->inputs->nb_rising || cvs->inputs->nb_opponents != cvs->inputs->nb_falling)
    {
        printf("We might have a problem here!!\n");
        printf("cvs->inputs->nb_opponents : %d \n", cvs->inputs->nb_opponents);
        printf("cvs->inputs->nb_rising : %d \n", cvs->inputs->nb_rising);
        printf("cvs->inputs->nb_falling: %d \n", cvs->inputs->nb_falling);

    }

    int index = cvs->inputs->falling_index;

    for (int i=0; i < cvs->inputs->nb_opponents; i++){

        // Conversion Angle
        double alpha1 = cvs->inputs->last_rising[index-i];
        double alpha2 = cvs->inputs->last_falling[index-i];
        printf("alpha1 : %f \n", alpha1);
        printf("alpha2 : %f \n", alpha2);

        // Angle wideness
        double angle = alpha2 - alpha1;
        if( angle < 0)
        {
            angle = (2*M_PI + angle);
        }

        if (angle > M_PI)
        {
            angle = (2*M_PI - angle);
        }

        // Target's center
        double diff = alpha2 - alpha1;
        double direction;

        if(diff > 0)
        {
            if(diff > M_PI){direction = alpha1 - (angle/2);}
            else{direction = alpha1 + (angle/2);}
        }
        else
        {
            if(diff < -M_PI){direction = alpha2 - (angle/2);}
            else{direction = alpha2 + (angle/2);}
        }

            // To change format
        double directionZeroTo2Pi = direction;
        if(directionZeroTo2Pi<0)
            {directionZeroTo2Pi = directionZeroTo2Pi + 2*M_PI;}


        // Target target's center Coming from val
        //double direction = alpha1 + (alpha/2);
        //if(direction>M_PI) {direction = direction - 2*M_PI;}

        double distance = (WIDENESS/2)/(tan(angle/2)); // distance from the center of the laser tower

        //with the 83mm offset from the tower
        double dist = sqrt(0.083*0.083+distance*distance-2*0.083*distance*cos(M_PI-direction));
        double dir = asin(distance*sin(M_PI-direction)/dist);
        double r=0.1935; //fixed for all the detected robot as the maximal radius of the possible positions

        // Result of the function in the robot frame
        cvs->pos_beacon_disdirray[0][i]=dist;
        cvs->pos_beacon_disdirray[1][i]=dir;
        cvs->pos_beacon_disdirray[2][i]=r;

        printf("pos_beacon_disdirray[0][%d] : %f \n",i, cvs->pos_beacon_disdirray[0][i]);
        printf("pos_beacon_disdirray[1][%d] : %f \n",i, cvs->pos_beacon_disdirray[1][i]);
        printf("pos_beacon_disdirray[2][%d] : %f \n",i, cvs->pos_beacon_disdirray[2][i]);
    }
}

NAMESPACE_CLOSE();
