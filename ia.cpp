#include "ia.h"
#include <stdio.h>
#include <stdlib.h>


void computecmd(CtrlStruct *theCtrlStruct)
{
    switch(theCtrlStruct->theUserStruct->state)
    {
    case 0: //en calibration
    {
        int wallnb = 0;

        //choisir une commande à donner au robot (en distance ou en vitesse)

        run_position(theCtrlStruct);//if the cmd is in terms of distance to travel (middle level controller)
        //theCtrlStruct->theUserStruct->wantedspeedl = ... //if cmd is in terms of speed
        //theCtrlStruct->theUserStruct->wantedspeedr = ...
        //if (on touche le mur) ...

        initpos(theCtrlStruct, wallnb);//put right the coord to 0;
        //at the end change state

        theCtrlStruct->theUserStruct->state = 2;
    }
    break;

    case 1: //action
        //
        /*
        ...
        */
        break;

    case 2: //navigation
    {
        double dest[2];
        //dest[0] = theCtrlStruct->theUserStruct->mymap->Node[theCtrlStruct->theUserStruct->mypath->obj[theCtrlStruct->theUserStruct->mypath->objnb]][0];
        //dest[1] = theCtrlStruct->theUserStruct->mymap->Node[theCtrlStruct->theUserStruct->mypath->obj[theCtrlStruct->theUserStruct->mypath->objnb]][1];
        //dest[2] = theCtrlStruct->theUserStruct->mymap->Node[theCtrlStruct->theUserStruct->mypath->obj[theCtrlStruct->theUserStruct->mypath->objnb]][2];

        middle_controller(theCtrlStruct, dest);//
    }
    break;

    case 3: //test
        //exemple
        theCtrlStruct->theUserStruct->wanteddist = 5;
        theCtrlStruct->theUserStruct->wantedangle = 0;
        run_position(theCtrlStruct);//calculate what tension to give to the wheels (cfr posctrl)
        break;

    default: //stop
        theCtrlStruct->theUserStruct->wantedspeedl = 0;
        theCtrlStruct->theUserStruct->wantedspeedr = 0;
        //change_state(); //could be interresting to create a fct that check if the state has to be changed
    }

}

void takenext_dest(CtrlStruct *theCtrlStruct, double dest[]) //function created to test but probably unuseful
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
    //
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

void middle_controller(CtrlStruct *structure, double objpos[3])
{
    double deltax = structure->theUserStruct->posxyt[0] - objpos[0];
    double deltay = structure->theUserStruct->posxyt[1] - objpos[1];
    double dist = sqrt(pow(deltax, 2)+pow(deltay,2));
    double diagangle = atan(((structure->theUserStruct->posxyt[1])-objpos[1])/((structure->theUserStruct->posxyt[0])-objpos[0]));//vérif angle < 0
    double diffangle = diagangle - (structure->theUserStruct->posxyt[2]);

    //moving state orientation ?
    if (fabs(diffangle) > M_PI/6 || fabs(dist) < 0.05) //needs to reoriente (if on obj pos or too desoriented
        structure->theUserStruct->movingState = 1;

    //apply
    if (structure->theUserStruct->movingState == 1)//mode orientation
    {
        rot(structure, diffangle);
        if (fabs(diffangle) < M_PI/20) //orientation almost perfect
            structure->theUserStruct->movingState = 0;
    }
    else
    {
        double centerangle = M_PI - (M_PI_2 - diffangle)*2; //triangle isocele
        double curv_radius = dist/(sin(centerangle/2))*2;//to be verified
        structure->theUserStruct->wantedspeedl = dist*KL*(curv_radius+WIDTH)/curv_radius;//to be verified (<0 ? if dist <0 ?)
        structure->theUserStruct->wantedspeedr = dist*KL*(curv_radius-WIDTH)/curv_radius;
    }
}

void rot(CtrlStruct *structure, double angle)
{
    structure->theUserStruct->wantedspeedl = KR*angle;
    structure->theUserStruct->wantedspeedr = -KR*angle;
}
