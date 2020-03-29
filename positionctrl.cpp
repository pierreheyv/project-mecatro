#include "positionctrl.h"
#include "IO/COM/SPI/SPI.hh"
#include "IO/COM/SPI/Specific/SPI_DE0.hh"

//#include <wiringPiSPI.h>
#include <time.h>

#define WR 0.03 //wheel radius (m)


// 1) getting informations

void updateinfo(CtrlStruct *theCtrlStruct, SPI_DE0 *spi)
{
    motorspeed(theCtrlStruct, spi); //compute speed @motor
    computeinfopos(theCtrlStruct, spi); //compute position (avancement, pos)@odometre (pas encore)
    computedisttobeam(theCtrlStruct, spi); //compute dist & angle towards detected beam
    needtoavoid(theCtrlStruct);
    proberror(theCtrlStruct);//calculate the error area and launch calibration if needed
}

void computeinfopos(CtrlStruct *theCtrlStruct,  SPI_DE0 *spi)
{

    int lenc = adjusttbit(spi->read(0x04));//left encoder adrs to be defined
    int renc = adjusttbit(spi->read(0x05));//right encoder adrs to be defined (nb tick total or tick/sec ?)

    //methode de calcul de la position à faire (en robotique on utilise la vitesse mais je sais pas si c'est le mieux)
    //transformer lenc & renc en tr/sec
    //ce qu'on fait en robotique (lenc & renc en tr/sec):

    double V = WR*(renc+lenc)/2;
    double W = WR*(renc-lenc)/2;

    double delta_t = clock() - theCtrlStruct->theUserStruct->stime;
    theCtrlStruct->theUserStruct->stime = (double) clock();

    double theta_inter = theCtrlStruct->theUserStruct->posxyt[2]+W*delta_t/2;

    theCtrlStruct->theUserStruct->posxyt[0] = theCtrlStruct->theUserStruct->posxyt[0] + V*cos(theta_inter);
    theCtrlStruct->theUserStruct->posxyt[1] = theCtrlStruct->theUserStruct->posxyt[0] + V*sin(theta_inter);
    theCtrlStruct->theUserStruct->posxyt[2] = theCtrlStruct->theUserStruct->posxyt[0] + W*delta_t;
}

void motorspeed(CtrlStruct *theCtrlStruct, SPI_DE0 *spi)
{

    int wer = adjusttbit(spi->read(0x00));
    int wel = adjusttbit(spi->read(0x01));

    if (wel != 0 && fabs(wel) < 1000)
        theCtrlStruct->theCtrlIn->l_wheel_speed = ((double) wel)*500/1750/5.6;
    else
        printf("readed valuel not updated\n");
    if (wer != 0 && fabs(wer) < 1000)
        theCtrlStruct->theCtrlIn->r_wheel_speed = ((double) wer)*500/1750/5.6;
    else
        printf("readed valuer == 0 not updated\n");
}

void computedisttobeam(CtrlStruct *theCtrlStruct, SPI_DE0 *spi)
{

    int lastRisingTick = adjusttbit(spi->read(0x02));
    int lastFallingTick = adjusttbit(spi->read(0x03));

    // Declaration of variables and constants
    int totalTicks = 1750;
    double pi = M_PI;   //M_PI;  //3.14159265359
    int wideness = 60;  //Wideness of the beacon in [mm]


    // Conversion Angle
    double alpha1 = ((2*pi)/totalTicks)*lastRisingTick;
    double alpha2 = ((2*pi)/totalTicks)*lastFallingTick;


    // Angle wideness
    double angle = alpha2 - alpha1;
    if( angle < 0)
    {
        angle = (2*pi + angle);
    }
    if (angle > M_PI)
    {
        angle = (2*pi - angle);
    }

    // Target target's center
    double diff = alpha2 - alpha1;
    double direction;

    if(diff > 0)
    {
        if(diff > pi)
        {
            direction = alpha1 - (angle/2);
        }
        else
        {
            direction = alpha1 + (angle/2);
        }
    }
    else
    {
        if(diff < -pi)
        {
            direction = alpha2 - (angle/2);
        }
        else
        {
            direction = alpha2 + (angle/2);
        }
    }

    // To change format
    double directionZeroTo2Pi = direction;
    if(directionZeroTo2Pi<0)
    {
        directionZeroTo2Pi = directionZeroTo2Pi + 2*pi;
    }

    // Distance from target
    double distance = (wideness/2)/(tan(angle/2));

    // Result of the function
    theCtrlStruct->theCtrlIn->angle = angle;
    theCtrlStruct->theCtrlIn->distance = distance;
    theCtrlStruct->theCtrlIn->direction = direction;
}

int adjusttbit (int beforeTreatment) //should be changed to manage bad reading
{
    int mask;
    if(beforeTreatment >= 0) //if positif number
    {
        mask = 0b00000000;
    }
    else
    {
        mask = 0b11111111;
    }

    return (mask <<24 | beforeTreatment>>8);
}

void needtoavoid(CtrlStruct *theCtrlStruct)
{
    if (theCtrlStruct->theCtrlIn->distance < DISTTOAVOIDR || theCtrlStruct->theUserStruct->posxyt[1] < 0.5)//si trop près d'un robot ou trop près d'un bord ...
        theCtrlStruct->theUserStruct->state = 4;//passing to potential field mode till obstacle avoided
}


void proberror(CtrlStruct *theCtrlStruct)
{
    //...
}
