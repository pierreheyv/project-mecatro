#include "speedctrl.h"
#include "positionctrl.h"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"


// 1) getting informations

void updateinfo(CtrlStruct *theCtrlStruct, int lenc1, int lenc2, int wls, int wrs){

    computeinfospeed(theCtrlStruct, wls, wrs); //compute speed @motor
    //computeinfopos(theCtrlStruct); //compute position (avancement, pos)@odometre (pas encore)
    //computedisttobeam(theCtrlStruct, lenc1, lenc2); //compute dist & angle to detected beam
}

void computeinfopos(CtrlStruct *theCtrlStruct) {
    //prendre le nombre de tick sur chaque roue et calculer l'avancement
    /*
    theCtrlStruct->theUserStruct->avancement[0] = 0;
    theCtrlStruct->theUserStruct->avancement[1] = 0;
    */
}

void computeinfospeed(CtrlStruct *theCtrlStruct, int wel, int wer) {
    if (wel != 0 && fabs(wel) < 1000)
        theCtrlStruct->theCtrlIn->l_wheel_speed = ((double) wel)*500/1750/5.6;
    else printf("readed valuel not updated\n");
    if (wer != 0 && fabs(wer) < 1000)
        theCtrlStruct->theCtrlIn->r_wheel_speed = ((double) wer)*500/1750/5.6;
    else printf("readed valuer == 0 not updated\n");

    printf("measured speed on left wheel : %f\n", theCtrlStruct->theCtrlIn->l_wheel_speed);

}

void computedisttobeam(CtrlStruct *theCtrlStruct, int lastRisingTick, int lastFallingTick) {

    // Declaration of variables and constants
    int totalTicks = 1750;
    double pi = M_PI;   //M_PI;  //3.14159265359
    int wideness = 60;  //Wideness of the beacon in [mm]


    // Conversion Angle
     double alpha1 = ((2*pi)/totalTicks)*lastRisingTick;
     double alpha2 = ((2*pi)/totalTicks)*lastFallingTick;


     // Angle wideness
     double angle = alpha2 - alpha1;
        if( angle < 0) {
               angle = (2*pi + angle);
        }
        if (angle > M_PI) {
               angle = (2*pi - angle);
        }

     // Target target's center

     double diff = alpha2 - alpha1;
     double direction;

     if(diff > 0){
        if(diff > pi){direction = alpha1 - (angle/2);}
        else{direction = alpha1 + (angle/2);}
     }
     else{
        if(diff < -pi){direction = alpha2 - (angle/2);}
        else{direction = alpha2 + (angle/2);}
     }

     // To change format
     double directionZeroTo2Pi = direction;
     if(directionZeroTo2Pi<0){directionZeroTo2Pi = directionZeroTo2Pi + 2*pi;}

     // Distance from target
    double distance = (wideness/2)/(tan(angle/2));

     // Result of the function

      theCtrlStruct->theCtrlIn->angle = angle;

      if (distance>3000){
        theCtrlStruct->theCtrlIn->distance = distance;
      }
      theCtrlStruct->theCtrlIn->direction = direction;

      printf("\n angle : %f \n", angle);
      printf("distance : %f \n", distance);
      printf("direction : %f \n", direction);
      printf("nb de tick 1 = %d 2= %d", lastRisingTick, lastFallingTick);
}

//2) Middlle controller

void run_position(CtrlStruct *theCtrlStruct){


    theCtrlStruct->theUserStruct->wantedspeedl = 1.25; //made in the main if loop!!
    theCtrlStruct->theUserStruct->wantedspeedr = 1.25;

    /*
    //next test is with a K factor
    theCtrlStruct->theUserStruct->wantedspeedl = K*theCtrlStruct->theUserStruct->distleft;
    theCtrlStruct->theUserStruct->wantedspeedr = K*theCtrlStruct->theUserStruct->distleft;
    //pour une rotation ?
    */
    //apply the speed
    run_speed_controller(theCtrlStruct);
}
