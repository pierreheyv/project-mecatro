#include "ia.h"
#include <stdio.h>
#include <stdlib.h>


void computecmd(CtrlStruct *theCtrlStruct)
{
    switch(theCtrlStruct->theUserStruct->state)
    {
    case 0: //en calibration
        int wallnb;
        //choisir une commande à donner au robot (en distance ou en vitesse)
        run_position(theCtrlStruct);//if the cmd is in terme of distance to travel (middle level controller)
        run_speed_controller(theCtrlStruct);//if the cmd is in terme of a wnted constant speed (directly to the low level)
        //if (on touche le mur) ...
        initpos(theCtrlStruct, wallnb);//put right the coord to 0;
        //at the end change state
        theCtrlStruct->theUserStruct->state = 2;
        break;

    case 1: //laisser là pour au cas ou on a besoin d'un state en plus (par exemple action)
        //
        /*
        ...
        */
        break;

    case 2: //mode de routine
        {
            double dest[2];
            pathplanning(theCtrlStruct);//...
            takenext_dest(theCtrlStruct, dest); //load dest with coordinate of the next destination
        }
        break;

    case 3: //mode de test
        //exemple
        theCtrlStruct->theUserStruct->wanteddist = 5;
        theCtrlStruct->theUserStruct->wantedangle = 0;
        run_position(theCtrlStruct);//calculate what tension to give to the wheels (cfr posctrl)
        break;

    default: //à l'arrêt
        theCtrlStruct->theUserStruct->wanteddist = 0;
        theCtrlStruct->theUserStruct->wantedangle = 0;
        run_position(theCtrlStruct);//calculate what tension to give to the wheels (cfr posctrl)
    }

}

void takenext_dest(CtrlStruct *theCtrlStruct, double dest[])
{
    //read from txt or table
    // if in a txt
    FILE *file;

    // use appropriate location if you are using MacOS or Linux
    file = fopen("dest.txt","r");

    if(file == NULL)
    {
        printf("dest.txt not openable");
        exit(1);
    }
    else
    {
        fscanf(file, "%lf %lf", &dest[0], &dest[1]);
        if (dest[0] == NULL || dest[1] == NULL)
            printf("end of list of dest or bad writing");
    }

    fclose(file);
}

void pathplanning(CtrlStruct *theCtrlStruct)
{
        //pathplanning ...
        /*
        ...
        */
}

void initpos(CtrlStruct *theCtrlStruct, int wallnb)
{
    if (wallnb == 0)
    {
        theCtrlStruct->theUserStruct->posxyt[0] = 0;//x = 0
    }
    else if (wallnb == 1)
    {
        theCtrlStruct->theUserStruct->posxyt[1] = 0;//y = 0
        theCtrlStruct->theUserStruct->posxyt[2] = 0;//orientation = 0
    }
}
